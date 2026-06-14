#!/usr/bin/env python3
"""Konwertuje BMP 24-bit na 16-bit RGB565 dla ESP32 TFT"""

from PIL import Image
import struct

def convert_bmp_to_rgb565(input_file, output_file):
    # Wczytaj obraz
    img = Image.open(input_file)
    img = img.convert('RGB')
    width, height = img.size
    
    print(f"Konwertowanie: {input_file} ({width}x{height})")
    print(f"  Format docelowy: 16-bit RGB565")
    
    # BMP Header
    with open(output_file, 'wb') as f:
        # BMP Header (14 bytes)
        f.write(b'BM')
        
        # Oblicz rozmiar
        row_size = width * 2
        row_size = (row_size + 3) & ~3
        pixel_data_size = row_size * height
        file_size = 14 + 40 + pixel_data_size
        
        f.write(struct.pack('<I', file_size))
        f.write(struct.pack('<HH', 0, 0))
        f.write(struct.pack('<I', 14 + 40))
        
        # DIB Header
        f.write(struct.pack('<I', 40))
        f.write(struct.pack('<i', width))
        f.write(struct.pack('<i', height))
        f.write(struct.pack('<HH', 1, 16))
        f.write(struct.pack('<I', 0))
        f.write(struct.pack('<I', pixel_data_size))
        f.write(struct.pack('<i', 2835))
        f.write(struct.pack('<i', 2835))
        f.write(struct.pack('<I', 0))
        f.write(struct.pack('<I', 0))
        
        # Pixel data (bottom-up)
        for y in range(height - 1, -1, -1):
            for x in range(width):
                r, g, b = img.getpixel((x, y))
                rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
                f.write(struct.pack('<H', rgb565))
            # Padding
            padding = (4 - (width * 2) % 4) % 4
            f.write(b'\x00' * padding)
    
    print(f"  Zapisano: {output_file}")

if __name__ == "__main__":
    convert_bmp_to_rgb565("data/mapa.bmp", "data/mapa.bmp")
    print("Konwersja zakończona!")
