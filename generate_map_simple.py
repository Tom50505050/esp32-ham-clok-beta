#!/usr/bin/env python3
"""
Generator mapy świata jako BMP 16-bit (RGB565) bez zewnętrznych bibliotek.
Używa tylko standardowej biblioteki Python.
"""

import struct
import math

def create_world_map_bmp(width=140, height=80, filename="world_map_140x80.bmp"):
    """Tworzy mapę świata jako BMP 16-bit RGB565"""
    
    # Kolory RGB565
    WATER_COLOR = (0x08, 0x10)  # Ciemnoniebieski: 0x0810 (R=0, G=16, B=16 -> 0b0000001000000100)
    LAND_COLOR = (0x14, 0xA3)   # Ciemnozielony: 0x14A3 (R=8, G=37, B=3 -> 0b0001010010100011)
    GRID_COLOR = (0x31, 0x46)   # Ciemnoszary: 0x3146 (R=24, G=36, B=6 -> 0b0011000101000110)
    
    # Konwersja RGB888 do RGB565
    def rgb888_to_rgb565(r, g, b):
        r5 = (r >> 3) & 0x1F
        g6 = (g >> 2) & 0x3F
        b5 = (b >> 3) & 0x1F
        return (r5 << 11) | (g6 << 5) | b5
    
    WATER = rgb888_to_rgb565(26, 40, 70)   # #1A2846
    LAND = rgb888_to_rgb565(45, 80, 35)    # #2D5023
    LAND_LIGHT = rgb888_to_rgb565(55, 95, 45)  # #375F2D
    GRID = rgb888_to_rgb565(60, 60, 60)    # #3C3C3C
    
    # Prosta definicja kontynentów jako wielokąty (współrzędne geograficzne)
    # Format: [(lon1, lat1), (lon2, lat2), ...]
    continents = [
        # Ameryka Północna (uproszczona)
        [(-170, 72), (-60, 72), (-55, 50), (-80, 25), (-120, 25), (-170, 50)],
        # Ameryka Południowa
        [(-85, 12), (-35, 12), (-35, -55), (-75, -55), (-85, -30)],
        # Europa
        [(-10, 70), (40, 70), (45, 35), (30, 35), (10, 45), (-10, 55)],
        # Afryka
        [(-20, 35), (50, 35), (52, -35), (15, -35), (-20, 10)],
        # Azja
        [(40, 72), (180, 70), (180, 10), (120, 10), (90, 20), (60, 35), (40, 50)],
        # Australia
        [(110, -10), (155, -10), (155, -45), (110, -45)],
        # Grenlandia
        [(-75, 85), (-20, 85), (-20, 60), (-55, 60)],
    ]
    
    def lon_lat_to_xy(lon, lat, w, h):
        """Konwertuje współrzędne geograficzne na piksele"""
        x = int((lon + 180) * w / 360)
        y = int((90 - lat) * h / 180)  # Odwrócone Y (90 na górze)
        return (max(0, min(w-1, x)), max(0, min(h-1, y)))
    
    def point_in_polygon(x, y, polygon):
        """Sprawdza czy punkt (x,y) jest wewnątrz wielokąta (algorytm ray casting)"""
        n = len(polygon)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside
    
    # Tworzenie bitmapy (wiersz po wierszu, od góry do dołu)
    # BMP wymaga wierszy od dołu do góry, więc odwrócimy na końcu
    pixels = []
    
    for row in range(height):
        lat = 90 - (row * 180 / height)  # Szerokość geograficzna dla tego wiersza
        row_pixels = []
        
        for col in range(width):
            lon = -180 + (col * 360 / width)  # Długość geograficzna dla tej kolumny
            
            # Sprawdź czy to ląd
            is_land = False
            for continent in continents:
                if point_in_polygon(lon, lat, continent):
                    is_land = True
                    break
            
            # Dodaj siatkę co 30 stopni
            is_grid = (abs(lon) % 30 < 2) or (abs(lat) % 30 < 2)
            
            if is_land:
                # Lekka tekstura dla lądu
                if (row % 7 == 0 and col % 5 == 0):
                    color = LAND_LIGHT
                else:
                    color = LAND
            elif is_grid:
                color = GRID
            else:
                color = WATER
            
            row_pixels.append(color)
        
        pixels.append(row_pixels)
    
    # Tworzenie pliku BMP 16-bit RGB565
    row_size = ((16 * width + 31) // 32) * 4  # Wiersze wyrównane do 4 bajtów
    pixel_data_size = row_size * height
    file_size = 14 + 40 + 12 + pixel_data_size
    
    # Nagłówek pliku (14 bajtów)
    header = struct.pack('<2sIHHI', 
        b'BM',           # Sygnatura
        file_size,       # Rozmiar pliku
        0,               # Reserved
        0,               # Reserved
        14 + 40 + 12     # Offset do danych obrazu
    )
    
    # BITMAPINFOHEADER (40 bajtów)
    dib_header = struct.pack('<IiiHHIIiiII',
        40,              # Rozmiar nagłówka
        width,           # Szerokość
        height,          # Wysokość
        1,               # Liczba płaszczyzn
        16,              # Bity na piksel
        3,               # Kompresja (BI_BITFIELDS dla RGB565)
        pixel_data_size, # Rozmiar danych obrazu
        2835,            # X pixels per meter (72 DPI)
        2835,            # Y pixels per meter (72 DPI)
        0,               # Kolory w palecie
        0                # Ważne kolory
    )
    
    # Maski bitowe dla RGB565 (12 bajtów)
    masks = struct.pack('<III',
        0xF800,          # Czerwony: 5 bitów (bity 11-15)
        0x07E0,          # Zielony: 6 bitów (bity 5-10)
        0x001F           # Niebieski: 5 bitów (bity 0-4)
    )
    
    # Zapis do pliku
    with open(filename, 'wb') as f:
        f.write(header)
        f.write(dib_header)
        f.write(masks)
        
        # Dane obrazu - od dołu do góry (format BMP)
        for y in range(height - 1, -1, -1):
            row_data = bytearray()
            for x in range(width):
                color = pixels[y][x]
                # Little endian: młodszy bajt pierwszy
                row_data.append(color & 0xFF)
                row_data.append((color >> 8) & 0xFF)
            
            f.write(row_data)
            
            # Padding do wyrównania do 4 bajtów
            padding = row_size - (width * 2)
            f.write(b'\x00' * padding)
    
    print(f"Wygenerowano: {filename}")
    print(f"  Rozmiar: {width}x{height}")
    print(f"  Format: BMP 16-bit RGB565")
    print(f"  Rozmiar pliku: {file_size} bajtów")


if __name__ == "__main__":
    # Generuj mapę dla popupu QRZ (140x80)
    create_world_map_bmp(140, 80, "world_map_140x80.bmp")
    
    # Generuj dużą mapę dla ekranu QRZ Info (320x180) - mniejsza aby zmieścić się w filesystem
    create_world_map_bmp(320, 180, "world_map_320x180.bmp")
