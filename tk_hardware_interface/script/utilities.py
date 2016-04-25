from struct import pack, unpack

def get_uint(reg_map, offset):
    return unpack('<I', reg_map[offset:offset+4])[0]

def get_int(reg_map, offset):
    return unpack('<i', reg_map[offset:offset+4])[0]

def set_uint(reg_map, offset, val):
    reg_map[offset:offset+4] = pack('<I', val)

def set_int(reg_map, offset, val):
    reg_map[offset:offset+4] = pack('<i', val)
