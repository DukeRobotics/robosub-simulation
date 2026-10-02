"""Run Stonefish on a container-local desktop and expose its window through noVNC."""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def main():
    width = int(os.environ.get('VIEW_WIDTH', '1280'))
    height = int(os.environ.get('VIEW_HEIGHT', '720'))
    if not 320 <= width <= 3840 or not 240 <= height <= 2160:
        raise ValueError('VIEW_WIDTH must be 320..3840 and VIEW_HEIGHT must be 240..2160')
    os.environ.update(DISPLAY=':99', SDL_VIDEODRIVER='x11', SDL_VIDEO_X11_WMCLASS='robosub_pool')
    os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
    os.environ.setdefault('LP_NUM_THREADS', '4')
    processes = []
    stopping = False
    simulator = None

    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    def start(command):
        process = subprocess.Popen(command, start_new_session=True)
        processes.append(process)
        return process

    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, stop)
    result = 0
    try:
        display = start(
            [
                'Xvfb',
                ':99',
                '-screen',
                '0',
                f'{width}x{height}x24',
                '+extension',
                'GLX',
                '+render',
                '-noreset',
                '-nolisten',
                'tcp',
            ]
        )
        for _ in range(100):
            if stopping or display.poll() is not None:
                raise RuntimeError('Virtual display stopped during startup')
            ready = subprocess.run(
                ['xdpyinfo', '-display', ':99'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
            )
            if ready.returncode == 0:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError('Virtual display did not become ready')
        start(['openbox', '--config-file', '/opt/simulation/openbox.xml'])
        start(
            [
                'x11vnc',
                '-display',
                ':99',
                '-localhost',
                '-rfbport',
                '5900',
                '-nopw',
                '-forever',
                '-shared',
                '-noxdamage',
                '-repeat',
                '-quiet',
            ]
        )
        start(['websockify', '--libserver', '--web', '/opt/simulation/viewer', '0.0.0.0:8080', '127.0.0.1:5900'])
        command = [
            'ros2',
            'launch',
            'robosub_simulation',
            'pool.launch.py',
            'keyboard_pulse_mode:=true',
            f'window_res_x:={width}',
            f'window_res_y:={height}',
            f'rendering_quality:={os.environ.get("RENDER_QUALITY", "low")}',
            f'render_rate:={os.environ.get("RENDER_RATE", "20")}',
        ]
        config = Path(os.environ.get('POOL_CONFIG', '/config/pool.yaml'))
        if config.exists():
            command.append(f'pool_config:={config}')
        robot_config = Path(os.environ.get('ROBOT_CONFIG', '/config/crush.yaml'))
        if robot_config.exists():
            command.append(f'robot_config:={robot_config}')
        simulator = start(command)
        print('Pool viewer: http://localhost:8080', flush=True)
        while not stopping:
            for process in processes:
                if process.poll() is not None:
                    result = process.returncode if process is simulator else 1
                    stopping = True
                    break
            time.sleep(0.2)
    except (OSError, RuntimeError) as error:
        print(f'Viewer startup failed: {error}', file=sys.stderr, flush=True)
        result = 1
    finally:
        # Stop ROS and physics before closing the display that owns the GL context.
        ordered = ([simulator] if simulator is not None else []) + [
            process for process in reversed(processes) if process is not simulator
        ]
        for process in ordered:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT if process is simulator else signal.SIGTERM)
                try:
                    process.wait(timeout=12 if process is simulator else 2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    return result


if __name__ == '__main__':
    sys.exit(main())
