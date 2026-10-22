#!/usr/bin/env python3
"""Lock physical Livox endpoints and exec the driver without a ros2-run parent."""
import argparse
import ctypes
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import sys


def endpoint_keys(config):
    """Keys do not depend on workspace, ROS domain, or robot namespace."""
    document = json.loads(Path(config).read_text())
    keys = {'sensor:' + str(item['ip']) for item in document.get('lidar_configs', []) if item.get('ip')}
    for section in document.values():
        if not isinstance(section, dict):
            continue
        host = section.get('host_net_info', {})
        if not isinstance(host, dict):
            continue
        for name, ip in host.items():
            if name.endswith('_ip') and ip:
                port = host.get(name[:-3] + '_port')
                if port:
                    keys.add(f'udp:{ip}:{port}')
    if not keys:
        raise ValueError('Driver config contains no sensor IP or host UDP endpoints')
    return keys


def conflicting_drivers(config=None):
    """Detect old/non-cooperating drivers too, even before ROS publishers exist."""
    target = endpoint_keys(config) if config is not None else None
    found = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            executable = os.readlink(process / 'exe').removesuffix(' (deleted)')
            if Path(executable).name != 'livox_ros_driver2_node':
                continue
            args = (process / 'cmdline').read_bytes().decode(errors='replace').split('\0')
            configs = [arg.split(':=', 1)[1] for arg in args if arg.startswith('user_config_path:=')]
            try:
                existing = endpoint_keys(configs[0]) if configs else None
            except (OSError, ValueError, KeyError, TypeError):
                existing = None
            if target is None or existing is None or target & existing:
                found.append(dict(pid=int(process.name), config=configs[0] if configs else 'unknown'))
        except (OSError, ValueError):
            continue
    return sorted(found, key=lambda item: item['pid'])


def lock_endpoints(config):
    directory = Path.home() / '.cache/co_3dto2d_mapping/driver_locks'
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    descriptors = []
    try:
        for key in sorted(endpoint_keys(config)):
            path = directory / (hashlib.sha256(key.encode()).hexdigest() + '.lock')
            fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
            descriptors.append(fd)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                owner = os.pread(fd, 2048, 0).decode(errors='replace')
                raise RuntimeError(f'LiDAR endpoint already locked: {key}; owner={owner}') from None
            os.set_inheritable(fd, True)
            payload = json.dumps(dict(pid=os.getpid(), endpoint=key, config=str(config))).encode()
            os.ftruncate(fd, 0)
            os.write(fd, payload)
        found = conflicting_drivers(config)
        if found:
            raise RuntimeError('Existing LiDAR driver process (including pre-publisher/orphan state): ' + json.dumps(found))
        return descriptors
    except BaseException:
        for fd in descriptors:
            os.close(fd)
        raise


def kill_on_parent_exit():
    # Linux retains PDEATHSIG across this exec. If the launcher crashes or is
    # killed, a driver stuck in SDK shutdown must not outlive that launcher.
    parent = os.getppid()
    expected = os.environ.get('LIVOX_LAUNCHER_PID')
    if expected is not None and parent != int(expected):
        raise RuntimeError('Driver launcher exited before driver initialization')
    libc = ctypes.CDLL(None, use_errno=True)
    libc.prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]
    libc.prctl.restype = ctypes.c_int
    if libc.prctl(1, int(signal.SIGKILL), 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    if os.getppid() != parent:
        raise RuntimeError('Driver launcher exited during startup')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not os.access(command[0], os.X_OK):
        raise ValueError('Driver executable missing or not executable')
    descriptors = lock_endpoints(args.config.resolve())
    try:
        kill_on_parent_exit()
        print(f'Livox driver guarded: pid={os.getpid()} config={args.config}; exclusive physical endpoints', flush=True)
        os.execv(command[0], command)
    finally:
        for fd in descriptors:
            os.close(fd)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as exc:
        print('Livox startup refused: ' + str(exc), file=sys.stderr)
        sys.exit(1)
