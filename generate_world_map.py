import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import struct

def create_16bit_bmp(width, height, filename):
    """Generuje mapę świata jako BMP 16-bit RGB565"""
    
    # 1. Pobranie danych mapy świata (uproszczone zarys kontynentów)
    fig, ax = plt.subplots(figsize=(width/100, height/100), dpi=100)
    
    try:
        from mpl_toolkits.basemap import Basemap
        m = Basemap(projection='cyl', resolution='c', ax=ax)
        m.drawcoastlines(color='white', linewidth=0.5)
        m.fillcontinents(color='#2d5016', lake_color='#1a3a5c')  # Ciemnozielone lądy
        m.drawmapboundary(fill_color='#1a3a5c')  # Ciemnoniebieskie morze
    except ImportError:
        # Fallback bez Basemap - rysujemy prostą siatkę
        ax.set_facecolor('#1a3a5c')
        # Rysujemy uproszczone kontury kontynentów
        continents = [
            # Ameryka Północna
            ([-130, -60, -60, -130], [25, 25, 70, 70]),
            # Ameryka Południowa
            ([-80, -35, -35, -80], [-55, -55, 10, 10]),
            # Europa
            ([-10, 40, 40, -10], [35, 35, 70, 70]),
            # Afryka
            ([-20, 50, 50, -20], [-35, -35, 35, 35]),
            # Azja
            ([40, 180, 180, 40], [10, 10, 70, 70]),
            # Australia
            ([110, 155, 155, 110], [-40, -40, -10, -10]),
        ]
        for lon, lat in continents:
            ax.fill(lon, lat, color='#2d5016', alpha=0.8)
    
    # Usuwamy osie
    ax.set_axis_off()
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    
    # Renderowanie do tablicy numpy
    fig.canvas.draw()
    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    plt.close(fig)

    # Zmiana rozmiaru do docelowego
    try:
        from PIL import Image
        img_pil = Image.fromarray(img).resize((width, height), Image.Resampling.LANCZOS)
        img_array = np.array(img_pil)
    except ImportError:
        # Prosty resize bez PIL
        from scipy.ndimage import zoom
        zoom_y = height / img.shape[0]
        zoom_x = width / img.shape[1]
        img_array = zoom(img, (zoom_y, zoom_x, 1), order=1).astype(np.uint8)

    # 2. Konwersja RGB888 -> RGB565 (16-bit)
    # R: 5 bitów, G: 6 bitów, B: 5 bitów
    r = (img_array[:, :, 0] >> 3).astype(np.uint16)
    g = (img_array[:, :, 1] >> 2).astype(np.uint16)
    b = (img_array[:, :, 2] >> 3).astype(np.uint16)
    rgb565 = (r << 11) | (g << 5) | b

    # 3. Tworzenie nagłówka BMP
    # Pliki 16-bit BMP wymagają definicji masek bitowych (BI_BITFIELDS)
    row_size = ((16 * width + 31) // 32) * 4
    pixel_data_size = row_size * height
    file_size = 14 + 40 + 12 + pixel_data_size  # Header + DIB + Masks + Data

    # Nagłówek pliku (14 bajtów)
    header = struct.pack('<2sIHHI', b'BM', file_size, 0, 0, 14 + 40 + 12)

    # Nagłówek DIB (BITMAPINFOHEADER - 40 bajtów)
    dib_header = struct.pack('<IiiHHIIiiII', 
        40, width, height, 1, 16, 3, pixel_data_size, 2835, 2835, 0, 0)

    # Maski dla RGB565 (12 bajtów)
    masks = struct.pack('<III', 0xF800, 0x07E0, 0x001F)

    # 4. Zapis do pliku
    with open(filename, 'wb') as f:
        f.write(header)
        f.write(dib_header)
        f.write(masks)
        # BMP zapisuje rzędy od dołu do góry
        for y in range(height - 1, -1, -1):
            row = rgb565[y, :]
            f.write(row.tobytes())
            # Padding do 4 bajtów
            padding = (row_size - (width * 2)) % 4
            f.write(b'\x00' * padding)

    print(f"Plik {filename} został wygenerowany pomyślnie.")
    print(f"Rozmiar: {width}x{height}, format: RGB565 16-bit")

if __name__ == "__main__":
    # Generujemy mapę o rozmiarze odpowiednim dla popupu (140x80)
    # lub większą do tła (480x320)
    create_16bit_bmp(140, 80, "world_map_popup.bmp")
    create_16bit_bmp(480, 320, "world_map_full.bmp")
