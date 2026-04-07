import os


def resolve_gpiochip() -> str:
    env_value = os.environ.get('OMNIBOT_GPIOCHIP', '').strip()
    candidates = []

    if env_value:
        candidates.append(env_value)

    # Raspberry Pi 5 / RP1 uses gpiochip4 for the external GPIO bank.
    candidates.extend([
        '/dev/gpiochip4',
        'gpiochip4',
        '/dev/gpiochip0',
        'gpiochip0',
    ])

    seen = set()
    for candidate in candidates:
        if not candidate or candidate in seen:
            continue
        seen.add(candidate)

        chip_path = candidate if candidate.startswith('/dev/') else f'/dev/{candidate}'
        if os.path.exists(chip_path):
            return chip_path

    raise RuntimeError(
        'No GPIO chip device found. Set OMNIBOT_GPIOCHIP to the correct /dev/gpiochipX path.'
    )