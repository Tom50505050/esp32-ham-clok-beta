#!/usr/bin/env python3
"""Konwertuje BMP 24-bit na 16-bit RGB565 dla ESP32 TFT"""

from PIL import Image
import struct
import sys

def convert_bmp_to_rgb565(input_file, output_file):
    # Wczytaj obraz
    img = Image.open(input_file)
    img = img.convert('RGB')
    width, height = img.size
    
    print(f"Konwertowanie: {input_file}")
    print(f"  Rozmiar: {width}x{height}")
    print(f"  Format źródłowy: 24-bit RGB")
    print(f"  Format docelowy: 16-bit RGB565")
    
    # BMP Header
    with open(output_file, 'wb') as f:
        # BMP Header (14 bytes)
        f.write(b'BM')  # Signature
        
        # Oblicz rozmiar pliku
        row_size = width * 2  # 2 bajty na piksel w RGB565
        row_size = (row_size + 3) & ~3  # Align do 4 bajtów
        pixel_data_size = row_size * height
        file_size = 14 + 40 + pixel_data_size
        
        f.write(struct.pack('<I', file_size))  # File size
        f.write(struct.pack('<HH', 0, 0))       # Reserved
        f.write(struct.pack('<I', 14 + 40))    # Data offset
        
        # DIB Header (BITMAPINFOHEADER, 40 bytes)
        f.write(struct.pack('<I', 40))          # DIB header size
        f.write(struct.pack('<i', width))      # Width
        f.write(struct.pack('<i', height))     # Height
        f.write(struct.pack('<HH', 1, 16))     # Planes, BPP
        f.write(struct.pack('<I', 0))          # Compression (0 = none)
        f.write(struct.pack('<I', pixel_data_size))  # Image size
        f.write(struct.pack('<i', 2835))       # X pixels per meter
        f.write(struct.pack('<i', 2835))       # Y pixels per meter
        f.write(struct.pack('<I', 0))           # Colors in color table
        f.write(struct.pack('<I', 0))           # Important colors
        
        # Pixel data (RGB565)
        # BMP zapisuje od dołu do góry (bottom-up)
        for y in range(height - 1, -1, -1):
            for x in range(width):
                r, g, b = img.getpixel((x, y))
                # Konwertuj na RGB565
                rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
                f.write(struct.pack('<H', rgb565))
            # Padding do 4 bajtów
            padding = (4 - (width * 2) % 4) % 4
            f.write(b'\x00' * padding)
    
    print(f"  Zapisano: {output_file}")
    print(f"  Rozmiar pliku: {file_size} bajtów")

if __name__ == "__main__":
    convert_bmp_to_rgb565("data/mapa.bmp", "data/mapa_16bit.bmp")
