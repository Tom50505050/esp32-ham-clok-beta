#!/usr/bin/env python3
"""
Generator mapy świata jako BMP 16-bit (RGB565) używając Cartopy.
Realistyczne kontury kontynentów z danych Natural Earth.
"""

import numpy as np
import matplotlib.pyplot as plt
import struct
import cartopy.crs as ccrs
import cartopy.feature as cfeature
from matplotlib.patches import Rectangle

def create_world_map_bmp_cartopy(width=140, height=80, filename="world_map_140x80.bmp"):
    """Tworzy realistyczną mapę świata jako BMP 16-bit RGB565 używając Cartopy"""
    
    # Tworzenie figury z odpowiednimi wymiarami
    fig = plt.figure(figsize=(width/100, height/100), dpi=100)
    ax = fig.add_subplot(1, 1, 1, projection=ccrs.PlateCarree())
    
    # Ustawienie zakresu (cały świat)
    ax.set_global()
    
    # Dodanie cech mapy
    # Ocean - niebieski
    ax.add_feature(cfeature.OCEAN, facecolor='#1a3a5c', edgecolor='none')
    # Lądy - zielone
    ax.add_feature(cfeature.LAND, facecolor='#2d5016', edgecolor='none')
    # Linie brzegowe
    ax.add_feature(cfeature.COASTLINE, linewidth=0.5, edgecolor='white')
    # Granice państw (subtelne)
    ax.add_feature(cfeature.BORDERS, linewidth=0.3, edgecolor='#1a3a5c', alpha=0.5)
    # Rzeki
    ax.add_feature(cfeature.RIVERS, linewidth=0.2, edgecolor='#1a3a5c', alpha=0.7)
    # Jeziora
    ax.add_feature(cfeature.LAKES, facecolor='#1a3a5c', edgecolor='none', alpha=0.8)
    
    # Siatka geograficzna (opcjonalna - subtelna)
    gl = ax.gridlines(draw_labels=False, linewidth=0.3, color='gray', alpha=0.3, linestyle='--')
    gl.xlines = True
    gl.ylines = True
    
    # Usunięcie osi
    ax.set_frame_on(False)
    
    # Renderowanie do tablicy numpy
    fig.canvas.draw()
    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    plt.close(fig)
    
    # Konwersja do PIL i resize do dokładnych wymiarów
    from PIL import Image
    img_pil = Image.fromarray(img).resize((width, height), Image.Resampling.LANCZOS)
    img_array = np.array(img_pil)
    
    # Konwersja RGB888 -> RGB565 (16-bit)
    r = (img_array[:, :, 0] >> 3).astype(np.uint16)
    g = (img_array[:, :, 1] >> 2).astype(np.uint16)
    b = (img_array[:, :, 2] >> 3).astype(np.uint16)
    rgb565 = (r << 11) | (g << 5) | b
    
    # Tworzenie nagłówka BMP 16-bit
    row_size = ((16 * width + 31) // 32) * 4
    pixel_data_size = row_size * height
    file_size = 14 + 40 + 12 + pixel_data_size
    
    # Nagłówek pliku (14 bajtów)
    header = struct.pack('<2sIHHI', b'BM', file_size, 0, 0, 14 + 40 + 12)
    
    # BITMAPINFOHEADER (40 bajtów)
    dib_header = struct.pack('<IiiHHIIiiII', 
        40, width, height, 1, 16, 3, pixel_data_size, 2835, 2835, 0, 0)
    
    # Maski dla RGB565 (12 bajtów)
    masks = struct.pack('<III', 0xF800, 0x07E0, 0x001F)
    
    # Zapis do pliku
    with open(filename, 'wb') as f:
        f.write(header)
        f.write(dib_header)
        f.write(masks)
        # BMP zapisuje od dołu do góry
        for y in range(height - 1, -1, -1):
            row = rgb565[y, :]
            f.write(row.tobytes())
            # Padding do 4 bajtów
            padding = (row_size - (width * 2)) % 4
            f.write(b'\x00' * padding)
    
    print(f"✓ Wygenerowano: {filename}")
    print(f"  Rozmiar: {width}x{height}")
    print(f"  Format: BMP 16-bit RGB565")
    print(f"  Rozmiar pliku: {file_size:,} bajtów ({file_size/1024:.1f} KB)")
    print(f"  Źródło danych: Natural Earth (przez Cartopy)")

if __name__ == "__main__":
    # Generuj mapę dla popupu QRZ
    create_world_map_bmp_cartopy(140, 80, "world_map_140x80.bmp")
    
    # Opcjonalnie: większa mapa do innych zastosowań
    # create_world_map_bmp_cartopy(480, 320, "world_map_480x320.bmp")
