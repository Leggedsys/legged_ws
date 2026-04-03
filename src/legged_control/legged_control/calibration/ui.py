"""Terminal UI helpers for the calibration script."""

import shutil
from colorama import Fore, Style, init as _colorama_init

_colorama_init(autoreset=True)

_ESTOP_LABEL: str = '[Ctrl+C=ESTOP]'


def set_estop_label(label: str) -> None:
    """Call after gamepad binding to show button name in prompts."""
    global _ESTOP_LABEL
    _ESTOP_LABEL = label


def phase_banner(number: int, title: str) -> None:
    width = shutil.get_terminal_size().columns
    bar = '─' * width
    print(f'\n{Fore.CYAN}{bar}')
    print(f'  Phase {number}: {title}')
    print(f'{bar}{Style.RESET_ALL}\n')


def info(msg: str) -> None:
    print(f'{Fore.WHITE}{msg}{Style.RESET_ALL}')


def success(msg: str) -> None:
    print(f'{Fore.GREEN}✓ {msg}{Style.RESET_ALL}')


def warn(msg: str) -> None:
    print(f'{Fore.YELLOW}⚠ {msg}{Style.RESET_ALL}')


def error(msg: str) -> None:
    print(f'{Fore.RED}✗ {msg}{Style.RESET_ALL}')


def prompt(msg: str) -> str:
    """Print a prompt with ESTOP reminder and return stripped input."""
    return input(f'{Fore.CYAN}{_ESTOP_LABEL} {msg}{Style.RESET_ALL} ').strip()


def confirm(msg: str) -> bool:
    """Ask a yes/no question. Returns True for 'y'."""
    answer = prompt(f'{msg} [y/n]:')
    return answer.lower() == 'y'


def joint_table(rows: list[tuple[str, float]]) -> None:
    """Print a two-column table: joint name | value (rad)."""
    print(f'\n  {"Joint":<14} {"Angle (rad)":>12}')
    print(f'  {"─"*14} {"─"*12}')
    for name, val in rows:
        print(f'  {name:<14} {val:>+12.4f}')
    print()
