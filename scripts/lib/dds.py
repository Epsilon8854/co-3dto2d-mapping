#!/usr/bin/env python3
"""Constrain Experiment 4 DDS to the inter-PC NIC before initializing ROS."""

from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))
import argparse
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'package.xml').is_file() and (p/'CMakeLists.txt').is_file())
POLICY = ROOT / 'config/dds.yaml'


def _ip_json(*args):
    return json.loads(subprocess.check_output(['ip', '-j', '-4', *args], text=True))


def _interface(selection):
    interfaces = _ip_json('address', 'show', 'up')
    addresses = {
        row['ifname']: [a['local'] for a in row.get('addr_info', [])
                        if a.get('family') == 'inet' and a.get('scope') == 'global']
        for row in interfaces
    }
    if selection == 'auto':
        routes = sorted(_ip_json('route', 'show', 'default'), key=lambda r: r.get('metric', 0))
        selection = next((r['dev'] for r in routes if addresses.get(r.get('dev'))), '')
        if not selection:
            raise ValueError('No IPv4 default-route NIC; set EXPERIMENT_04_DDS_INTERFACE to the inter-PC NIC or its IP')
    if selection in addresses and addresses[selection]:
        if len(addresses[selection]) != 1:
            raise ValueError('NIC has multiple IPv4 addresses; set EXPERIMENT_04_DDS_INTERFACE to the intended IP')
        return selection, addresses[selection][0]
    for name, ips in addresses.items():
        if selection in ips:
            return name, str(ipaddress.IPv4Address(selection))
    raise ValueError(f'Inter-PC interface/IP is not active: {selection}')


def _kernel_limit(name):
    return int((Path('/proc/sys/net/core') / name).read_text())


def _xml(address, buffer_bytes, shm_bytes):
    root = ET.Element('profiles', xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    descriptors = ET.SubElement(root, 'transport_descriptors')
    udp = ET.SubElement(descriptors, 'transport_descriptor')
    for key, value in [('transport_id', 'experiment04_udp'), ('type', 'UDPv4'),
                       ('sendBufferSize', buffer_bytes), ('receiveBufferSize', buffer_bytes)]:
        ET.SubElement(udp, key).text = str(value)
    allow = ET.SubElement(udp, 'interfaceWhiteList')
    ET.SubElement(allow, 'address').text = address
    # Preserve blocking UDP semantics; asynchronous publication is configured
    # separately. Do not silently turn socket overflow into successful sends.
    ET.SubElement(udp, 'non_blocking_send').text = 'false'
    shm = ET.SubElement(descriptors, 'transport_descriptor')
    for key, value in [('transport_id', 'experiment04_shm'), ('type', 'SHM'), ('segment_size', shm_bytes)]:
        ET.SubElement(shm, key).text = str(value)
    participant = ET.SubElement(root, 'participant', profile_name='experiment04_network', is_default_profile='true')
    rtps = ET.SubElement(participant, 'rtps')
    transports = ET.SubElement(rtps, 'userTransports')
    for name in ('experiment04_udp', 'experiment04_shm'):
        ET.SubElement(transports, 'transport_id').text = name
    # Otherwise Fast DDS also enables an unrestricted built-in UDP transport.
    ET.SubElement(rtps, 'useBuiltinTransports').text = 'false'
    ET.indent(root)
    return ET.tostring(root, encoding='utf-8', xml_declaration=True)


def prepare_environment():
    """Must run before rclpy.init(), ROS discovery, or starting any ROS child."""
    mode = os.environ.get('EXPERIMENT_04_DDS_MODE', 'managed')
    if mode == 'external':
        info = dict(mode=mode, rmw=os.environ.get('RMW_IMPLEMENTATION', 'default'),
                    profile=os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE', '')))
        return info
    if mode != 'managed':
        raise ValueError('EXPERIMENT_04_DDS_MODE must be managed or external')
    rmw = os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    if rmw != 'rmw_fastrtps_cpp':
        raise ValueError(f'Existing RMW {rmw}; use EXPERIMENT_04_DDS_MODE=external for a separately managed DDS configuration')
    cache = Path.home() / '.cache/co_3dto2d_mapping/dds'
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        existing = os.environ.get(key)
        if existing and Path(existing).resolve().parent != cache.resolve():
            raise ValueError(f'{key} already points to an external profile; use EXPERIMENT_04_DDS_MODE=external to preserve it')
    if os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML', '0') != '0':
        raise ValueError('Existing XML writer QoS override; use EXPERIMENT_04_DDS_MODE=external to preserve it')
    policy = yaml.safe_load(POLICY.read_text())
    interface, address = _interface(os.environ.get('EXPERIMENT_04_DDS_INTERFACE', policy['interface']))
    requested = int(policy['udp_socket_buffer_bytes'])
    send_max, receive_max = _kernel_limit('wmem_max'), _kernel_limit('rmem_max')
    buffer_bytes = min(requested, send_max, receive_max)
    shm_bytes = int(policy['shared_memory_segment_bytes'])
    if buffer_bytes < 65536 or shm_bytes < 1048576:
        raise ValueError('DDS UDP buffer must be >=65536 and SHM segment >=1048576 bytes')
    publication = os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE', policy['publication_mode'])
    if publication not in ('ASYNCHRONOUS', 'SYNCHRONOUS', 'AUTO'):
        raise ValueError('Invalid RMW_FASTRTPS_PUBLICATION_MODE')
    data = _xml(address, buffer_bytes, shm_bytes)
    digest = hashlib.sha256(data).hexdigest()
    cache.mkdir(parents=True, exist_ok=True)
    profile = cache / (digest[:20] + '.xml')
    if not profile.exists():
        with tempfile.NamedTemporaryFile(dir=cache, delete=False) as handle:
            handle.write(data)
            temporary = Path(handle.name)
        temporary.replace(profile)
    os.environ.update(RMW_IMPLEMENTATION=rmw,
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
                      FASTDDS_DEFAULT_PROFILES_FILE=str(profile),
                      RMW_FASTRTPS_PUBLICATION_MODE=publication)
    info = dict(mode=mode, rmw=rmw, interface=interface, ipv4=address,
                profile=str(profile), profile_sha256=digest, publication_mode=publication,
                udp_buffer_requested_bytes=requested, udp_buffer_configured_bytes=buffer_bytes,
                os_wmem_max=send_max, os_rmem_max=receive_max, shared_memory_segment_bytes=shm_bytes,
                non_blocking_send=False)
    print(f'[DDS] {interface}={address} only + local SHM; publication={publication}; UDP buffer request={buffer_bytes} bytes', file=sys.stderr, flush=True)
    if buffer_bytes < requested:
        print('[DDS] UDP buffer capped by current OS limits; no kernel settings were changed.', file=sys.stderr, flush=True)
    return info


def snapshot_environment(directory, info):
    """Save the effective XML; a repository policy alone cannot identify NIC/IP."""
    profile = info.get('profile')
    if profile and Path(profile).is_file():
        (Path(directory) / 'dds_profile.xml').write_bytes(Path(profile).read_bytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--describe', action='store_true')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    info = prepare_environment()
    if args.describe:
        print(json.dumps(info, indent=2)); return
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('Pass --describe or -- COMMAND [ARGS...]')
    os.execvpe(command[0], command, os.environ)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f'DDS setup failed: {exc}', file=sys.stderr)
        sys.exit(1)
