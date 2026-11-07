#!/usr/bin/env python3
"""
Compute DroneCAN (UAVCAN v0) data type signatures from DSDL files.
Self-contained re-implementation of the signature rules (CRC-64-WE over the
normalized definition, extended with nested type signatures), so we don't
depend on pydronecan at build time.

usage: tools/dsdl_signature.py <dsdl_root> uavcan.equipment.ahrs.RawIMU [...]
"""
import os, re, struct, sys

MASK = 0xFFFFFFFFFFFFFFFF
POLY = 0x42F0E1EBA9EA3693

class Sig:
    def __init__(self, extend=None):
        self.crc = MASK if extend is None else (extend & MASK) ^ MASK
    def add(self, data):
        if isinstance(data, str):
            data = data.encode()
        for b in data:
            self.crc ^= (b << 56) & MASK
            for _ in range(8):
                self.crc = ((self.crc << 1) & MASK) ^ POLY if self.crc & (1 << 63) else (self.crc << 1) & MASK
    def value(self):
        return (self.crc & MASK) ^ MASK

_s = Sig(); _s.add(b'123'); _s.add('456789'); assert _s.value() == 0x62EC59E3F1A4F00A

PRIM = re.compile(r'^(bool|u?int\d+|float\d+|void\d+)$')

def find_file(root, full):
    ns, short = full.rsplit('.', 1)
    d = os.path.join(root, *ns.split('.'))
    for f in os.listdir(d):
        parts = f.split('.')
        if f.endswith('.uavcan') and parts[-2] == short:
            return os.path.join(d, f)
    raise FileNotFoundError(full)

_cache = {}

def type_signature(root, full):
    if full in _cache:
        return _cache[full]
    ns = full.rsplit('.', 1)[0]
    lines = []
    nested = []
    for raw in open(find_file(root, full)):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        if line == '@union':
            lines.append('@union'); continue
        if line == '---':
            lines.append('---'); continue
        if re.match(r'^(?:saturated\s+|truncated\s+)?[\w.]+\s+\w+\s*=', line):
            continue  # constants are not part of the signature source of messages
        toks = line.split()
        cast = 'saturated'
        if toks[0] in ('saturated', 'truncated'):
            cast = toks.pop(0)
        tdef = toks[0]
        name = toks[1] if len(toks) > 1 else None
        m = re.match(r'^([\w.]+)(\[.*\])?$', tdef)
        base, arr = m.group(1), m.group(2) or ''
        if PRIM.match(base):
            norm = base if base.startswith('void') else f'{cast} {base}'
            if base == 'bool':
                norm = f'{cast} bool'
        else:
            full_nested = base if '.' in base else f'{ns}.{base}'
            norm = full_nested
            nested.append(full_nested)
        norm += arr
        lines.append(norm if name is None else f'{norm} {name}')
    src = full + '\n' + '\n'.join(lines) + '\n'
    src = src.strip().replace('\n\n\n', '\n').replace('\n\n', '\n')
    s = Sig(); s.add(src)
    sig = Sig(s.value())
    for n in nested:
        v = sig.value()
        sig.add(struct.pack('<Q', type_signature(root, n)))
        sig.add(struct.pack('<Q', v))
    _cache[full] = sig.value()
    return _cache[full]

if __name__ == '__main__':
    root = sys.argv[1]
    for t in sys.argv[2:]:
        print(f'{t} 0x{type_signature(root, t):016X}')
