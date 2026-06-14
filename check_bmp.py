import struct
with open('data/mapa.bmp','rb') as f:
    h = f.read(54)
    bpp = struct.unpack('<H', h[28:30])[0]
    w = struct.unpack('<i', h[18:22])[0]
    hgt = struct.unpack('<i', h[22:26])[0]
    print(f'Format mapa.bmp: {bpp}-bit RGB')
    print(f'Rozmiar: {w}x{hgt}')
