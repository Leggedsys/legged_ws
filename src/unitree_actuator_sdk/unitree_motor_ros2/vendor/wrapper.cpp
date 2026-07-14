#include <pybind11/pybind11.h>
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

#include <chrono>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace py = pybind11;

// Drop-in replacement for SerialPort::sendRecv with a REAL recv timeout.
//
// The prebuilt libUnitreeMotorSDK hardcodes a 20 ms recv timeout internally:
// both the SerialPort(timeOutUs=...) constructor argument and IOPort::resetIO
// are ignored by its recv path (measured 2026-07-14). A normal reply lands in
// <0.3 ms at 4 Mbps, so on a lossy bus every missed reply stalls the polling
// loop ~80x longer than a successful exchange.
//
// Frame encode/decode (modify_data / extract_data, incl. CRC) still comes
// from the closed lib; only the serial I/O + wait is reimplemented here.
class FastSerialPort {
public:
    explicit FastSerialPort(const std::string &portName, size_t timeOutUs = 1500)
        : _timeout_us(timeOutUs) {
        _fd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);
        if (_fd < 0)
            throw std::runtime_error("FastSerialPort: cannot open " + portName);
        struct termios tio;
        tcgetattr(_fd, &tio);
        cfmakeraw(&tio);
        cfsetispeed(&tio, B4000000);
        cfsetospeed(&tio, B4000000);
        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        tcsetattr(_fd, TCSANOW, &tio);
        // USB-serial drivers batch RX into ~16 ms windows without this flag.
        struct serial_struct ss;
        if (ioctl(_fd, TIOCGSERIAL, &ss) == 0) {
            ss.flags |= ASYNC_LOW_LATENCY;
            ioctl(_fd, TIOCSSERIAL, &ss);
        }
        tcflush(_fd, TCIOFLUSH);
    }
    ~FastSerialPort() {
        if (_fd >= 0) ::close(_fd);
    }
    void setTimeout(size_t timeOutUs) { _timeout_us = timeOutUs; }

    bool sendRecv(MotorCmd *cmd, MotorData *data) {
        data->correct = false;
        cmd->modify_data(cmd);
        int txlen = cmd->hex_len > 0 ? cmd->hex_len : 17;  // GO cmd frame: 17 B
        // Drop stale bytes (late/truncated earlier replies) before the exchange
        // so they can't be mis-parsed as this motor's answer.
        tcflush(_fd, TCIFLUSH);
        if (::write(_fd, cmd->get_motor_send_data(), txlen) != txlen)
            return false;
        uint8_t *rx = data->get_motor_recv_data();
        const size_t want = 16;  // GO feedback frame: 16 B
        size_t got = 0;
        auto t0 = std::chrono::steady_clock::now();
        while (got < want) {
            long elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - t0)
                               .count();
            long remain = static_cast<long>(_timeout_us) - elapsed;
            if (remain <= 0) return false;
            fd_set rset;
            FD_ZERO(&rset);
            FD_SET(_fd, &rset);
            struct timeval tv;
            tv.tv_sec = remain / 1000000;
            tv.tv_usec = remain % 1000000;
            if (::select(_fd + 1, &rset, nullptr, nullptr, &tv) <= 0)
                return false;
            ssize_t n = ::read(_fd, rx + got, want - got);
            if (n > 0) got += static_cast<size_t>(n);
        }
        data->hex_len = static_cast<int>(want);
        return data->extract_data(data);
    }

private:
    int _fd;
    size_t _timeout_us;
};

PYBIND11_MODULE(_unitree_actuator_sdk, m){ 
m.doc() = "unitree_actuator_sdk python wrapper"; 
 
py::enum_<MotorType>(m, "MotorType")
  .value("A1", MotorType::A1, "A1 Motor")
  .value("B1", MotorType::B1, "B1 Motor")
  .value("GO_M8010_6", MotorType::GO_M8010_6, "GO_M8010_6 Motor");

py::enum_<MotorMode>(m, "MotorMode")
  .value("BRAKE", MotorMode::BRAKE, "BRAKE mode")
  .value("FOC", MotorMode::FOC, "FOC")
  .value("CALIBRATE", MotorMode::CALIBRATE, "CALIBRATE mode");

py::class_<MotorCmd>(m, "MotorCmd") 
  .def(pybind11::init<>())
  .def_readwrite("motorType", &MotorCmd::motorType)
  .def_readwrite("hex_len", &MotorCmd::hex_len)
  .def_readwrite("id", &MotorCmd::id)
  .def_readwrite("mode", &MotorCmd::mode)
  .def_readwrite("tau", &MotorCmd::tau)
  .def_readwrite("dq", &MotorCmd::dq)
  .def_readwrite("q", &MotorCmd::q)
  .def_readwrite("kp", &MotorCmd::kp)
  .def_readwrite("kd", &MotorCmd::kd);

py::class_<MotorData>(m, "MotorData") 
  .def(pybind11::init<>())
  .def_readwrite("motorType", &MotorData::motorType)
  .def_readwrite("hex_len", &MotorData::hex_len)
  .def_readwrite("motor_id", &MotorData::motor_id)
  .def_readwrite("mode", &MotorData::mode)
  .def_readwrite("temp", &MotorData::temp)
  .def_readwrite("merror", &MotorData::merror)
  .def_readwrite("tau", &MotorData::tau)
  .def_readwrite("dq", &MotorData::dq)
  .def_readwrite("q", &MotorData::q)
  .def_readwrite("correct", &MotorData::correct);

py::class_<SerialPort>(m, "SerialPort")
  .def(py::init<const std::string &>())
  .def("test", &SerialPort::test)
  .def("sendRecv", py::overload_cast<MotorCmd*, MotorData*>(&SerialPort::sendRecv));

py::class_<FastSerialPort>(m, "FastSerialPort")
  .def(py::init<const std::string &, size_t>(),
       py::arg("portName"), py::arg("timeOutUs") = 1500)
  .def("setTimeout", &FastSerialPort::setTimeout, py::arg("timeOutUs"))
  .def("sendRecv", &FastSerialPort::sendRecv,
       py::call_guard<py::gil_scoped_release>());

m.def("queryMotorMode", &queryMotorMode, "Query Motor Mode");
m.def("queryGearRatio", &queryGearRatio, "Query Motor Gear Ratio");
}
