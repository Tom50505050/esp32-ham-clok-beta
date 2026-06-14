#!/usr/bin/env python3
"""
Generator mapy świata jako BMP 16-bit (RGB565) z prawdziwymi konturami kontynentów.
Pobiera dane Natural Earth z internetu.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Używamy non-interactive backend
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import struct
import urllib.request
import json
import zipfile
import io

def download_natural_earth_data():
    """Pobiera uproszczone dane konturów kontynentów"""
    # Używamy publicznego endpointu geojson
    url = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_110m_land.geojson"
    
    try:
        print("Pobieranie danych Natural Earth...")
        with urllib.request.urlopen(url, timeout=30) as response:
            data = json.loads(response.read().decode('utf-8'))
        print("✓ Dane pobrane pomyślnie")
        return data
    except Exception as e:
        print(f"Błąd pobierania: {e}")
        print("Używam uproszczonych danych wbudowanych...")
        return None

def create_world_map_bmp_real(width=140, height=80, filename="world_map_140x80.bmp"):
    """Tworzy realistyczną mapę świata jako BMP 16-bit RGB565"""
    
    # Tworzenie figury
    fig, ax = plt.subplots(figsize=(width/100, height/100), dpi=100)
    
    # Tło oceanu
    ax.set_facecolor('#1a3a5c')
    
    # Pobierz dane kontynentów
    geo_data = download_natural_earth_data()
    
    if geo_data and 'features' in geo_data:
        # Rysuj kontynenty z danych geojson
        for feature in geo_data['features']:
            geometry = feature.get('geometry', {})
            if geometry.get('type') == 'Polygon':
                coords = geometry['coordinates'][0]
                lons = [c[0] for c in coords]
                lats = [c[1] for c in coords]
                ax.fill(lons, lats, color='#2d5016', edgecolor='#3d6026', linewidth=0.3)
            elif geometry.get('type') == 'MultiPolygon':
                for polygon in geometry['coordinates']:
                    coords = polygon[0]
                    lons = [c[0] for c in coords]
                    lats = [c[1] for c in coords]
                    ax.fill(lons, lats, color='#2d5016', edgecolor='#3d6026', linewidth=0.3)
    else:
        # Fallback - rysuj uproszczone kontury
        print("Rysowanie uproszczonych konturów...")
        continents = [
            # Ameryka Północna
            ([-170, -50, -50, -170], [15, 15, 75, 75]),
            # Ameryka Południowa  
            ([-85, -35, -35, -85], [-55, -55, 15, 15]),
            # Europa
            ([-15, 45, 45, -15], [35, 35, 72, 72]),
            # Afryka
            ([-20, 55, 55, -20], [-35, -35, 38, 38]),
            # Azja
            ([40, 180, 180, 40], [5, 5, 77, 77]),
            # Australia
            ([110, 160, 160, 110], [-45, -45, -10, -10]),
            # Grenlandia
            ([-75, -12, -12, -75], [58, 58, 85, 85]),
        ]
        for lons, lats in continents:
            ax.fill(lons, lats, color='#2d5016', edgecolor='#3d6026', linewidth=0.3)
    
    # Ustawienia osi
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Siatka geograficzna (subtelna)
    for lat in range(-60, 61, 30):
        ax.axhline(y=lat, color='#3d4a6b', linewidth=0.2, alpha=0.5)
    for lon in range(-120, 121, 60):
        ax.axvline(x=lon, color='#3d4a6b', linewidth=0.2, alpha=0.5)
    
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    
    # Renderowanie
    fig.canvas.draw()
    buf = fig.canvas.buffer_rgba()
    img = np.asarray(buf)[:, :, :3]  # Bierzemy tylko RGB, pomijamy alpha
    plt.close(fig)
    
    # Konwersja do PIL
    from PIL import Image
    img_pil = Image.fromarray(img).resize((width, height), Image.Resampling.LANCZOS)
    img_array = np.array(img_pil)
    
    # Konwersja RGB888 -> RGB565
    r = (img_array[:, :, 0] >> 3).astype(np.uint16)
    g = (img_array[:, :, 1] >> 2).astype(np.uint16)
    b = (img_array[:, :, 2] >> 3).astype(np.uint16)
    rgb565 = (r << 11) | (g << 5) | b
    
    # Nagłówek BMP
    row_size = ((16 * width + 31) // 32) * 4
    pixel_data_size = row_size * height
    file_size = 14 + 40 + 12 + pixel_data_size
    
    header = struct.pack('<2sIHHI', b'BM', file_size, 0, 0, 14 + 40 + 12)
    dib_header = struct.pack('<IiiHHIIiiII', 
        40, width, height, 1, 16, 3, pixel_data_size, 2835, 2835, 0, 0)
    masks = struct.pack('<III', 0xF800, 0x07E0, 0x001F)
    
    # Zapis
    with open(filename, 'wb') as f:
        f.write(header)
        f.write(dib_header)
        f.write(masks)
        for y in range(height - 1, -1, -1):
            row = rgb565[y, :]
            f.write(row.tobytes())
            padding = (row_size - (width * 2)) % 4
            f.write(b'\x00' * padding)
    
    print(f"OK Wygenerowano: {filename}")
    print(f"  Rozmiar: {width}x{height}")
    print(f"  Format: BMP 16-bit RGB565")
    print(f"  Rozmiar pliku: {file_size:,} bajtów ({file_size/1024:.1f} KB)")

if __name__ == "__main__":
    create_world_map_bmp_real(140, 80, "world_map_140x80.bmp")
