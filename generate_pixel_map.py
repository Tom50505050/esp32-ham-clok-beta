#!/usr/bin/env python3
"""
Generator mapy świata w stylu retro pixel art dla QRZ popup
Amateur Radio DX World Map - 16-Bit Edition
"""

from PIL import Image, ImageDraw, ImageFont
import struct

def create_pixel_world_map(width, height, filename):
    """Tworzy mapę świata w stylu retro pixel art"""
    
    # Kolory w formacie RGB565
    OCEAN = (0x00, 0x1F, 0xD4)  # Głęboki niebieski ocean
    LAND_LIGHT = (0x4A, 0x90, 0x4A)  # Jasnozielony ląd
    LAND_DARK = (0x2E, 0x5A, 0x2E)   # Ciemnozielony ląd
    LAND_TAN = (0xD4, 0xA5, 0x4A)   # Beżowy (pustynie)
    LAND_BROWN = (0x8B, 0x45, 0x13) # Brązowy (góry)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GRID = (0x1E, 0x90, 0xFF)  # Jasny niebieski dla siatki
    
    img = Image.new('RGB', (width, height), OCEAN)
    draw = ImageDraw.Draw(img)
    
    # Proporcje dla różnych rozmiarów
    scale_x = width / 480
    scale_y = height / 320
    
    def sx(x): return int(x * scale_x)
    def sy(y): return int(y * scale_y)
    
    # Ameryka Północna
    draw.polygon([
        (sx(20), sy(60)), (sx(80), sy(40)), (sx(140), sy(50)),
        (sx(160), sy(80)), (sx(140), sy(120)), (sx(100), sy(130)),
        (sx(60), sy(110)), (sx(30), sy(90))
    ], fill=LAND_LIGHT, outline=LAND_DARK)
    
    # Ameryka Południowa
    draw.polygon([
        (sx(100), sy(130)), (sx(140), sy(140)), (sx(160), sy(180)),
        (sx(140), sy(240)), (sx(110), sy(260)), (sx(90), sy(200))
    ], fill=LAND_LIGHT, outline=LAND_DARK)
    
    # Europa
    draw.polygon([
        (sx(220), sy(70)), (sx(260), sy(60)), (sx(280), sy(80)),
        (sx(270), sy(100)), (sx(240), sy(105)), (sx(220), sy(90))
    ], fill=LAND_LIGHT, outline=LAND_DARK)
    
    # Afryka
    draw.polygon([
        (sx(230), sy(110)), (sx(280), sy(115)), (sx(300), sy(150)),
        (sx(290), sy(210)), (sx(250), sy(240)), (sx(220), sy(190))
    ], fill=LAND_TAN, outline=LAND_DARK)
    
    # Azja
    draw.polygon([
        (sx(280), sy(60)), (sx(380), sy(50)), (sx(420), sy(70)),
        (sx(430), sy(110)), (sx(400), sy(140)), (sx(340), sy(130)),
        (sx(300), sy(110)), (sx(290), sy(80))
    ], fill=LAND_LIGHT, outline=LAND_DARK)
    
    # Australia
    draw.polygon([
        (sx(360), sy(200)), (sx(420), sy(190)), (sx(440), sy(220)),
        (sx(420), sy(250)), (sx(370), sy(245))
    ], fill=LAND_TAN, outline=LAND_DARK)
    
    # Grenlandia
    draw.polygon([
        (sx(180), sy(30)), (sx(220), sy(20)), (sx(230), sy(50)),
        (sx(200), sy(55))
    ], fill=LAND_LIGHT, outline=LAND_DARK)
    
    # Antarktyda
    draw.polygon([
        (sx(50), sy(290)), (sx(200), sy(295)), (sx(350), sy(290)),
        (sx(400), sy(300)), (sx(250), sy(310)), (sx(100), sy(305))
    ], fill=LAND_LIGHT, outline=LAND_DARK)
    
    # Japonia
    draw.polygon([
        (sx(420), sy(85)), (sx(440), sy(80)), (sx(445), sy(95)), (sx(430), sy(100))
    ], fill=LAND_LIGHT, outline=LAND_DARK)
    
    # Wielka Brytania
    draw.polygon([
        (sx(210), sy(75)), (sx(220), sy(72)), (sx(218), sy(82)), (sx(208), sy(80))
    ], fill=LAND_LIGHT, outline=LAND_DARK)
    
    # Nowa Zelandia
    draw.polygon([
        (sx(450), sy(240)), (sx(465), sy(235)), (sx(468), sy(250)), (sx(455), sy(255))
    ], fill=LAND_LIGHT, outline=LAND_DARK)
    
    # Rysuj siatkę geograficzną
    for i in range(1, 6):
        # Linie pionowe (długości geograficzne)
        x = sx(i * 80)
        draw.line([(x, sy(10)), (x, sy(310))], fill=GRID, width=1)
        
        # Linie poziome (szerokości geograficzne)
        y = sy(i * 52)
        draw.line([(sx(10), y), (sx(470), y)], fill=GRID, width=1)
    
    # Ramka
    draw.rectangle([sx(0), sy(0), sx(479), sy(319)], outline=WHITE, width=2)
    
    # Dodaj etykiety ITU zones (opcjonalnie - uproszczone)
    try:
        font = ImageFont.truetype("arial.ttf", int(12 * scale_y))
    except:
        font = ImageFont.load_default()
    
    # Zapisz jako BMP 16-bit RGB565
    img_rgb565 = img.convert('RGB')
    
    with open(filename, 'wb') as f:
        # BMP Header
        f.write(b'BM')
        file_size = 14 + 40 + width * height * 2
        f.write(struct.pack('<I', file_size))
        f.write(struct.pack('<HH', 0, 0))
        f.write(struct.pack('<I', 14 + 40))
        
        # DIB Header (BITMAPINFOHEADER)
        f.write(struct.pack('<I', 40))
        f.write(struct.pack('<i', width))
        f.write(struct.pack('<i', height))
        f.write(struct.pack('<HH', 1, 16))
        f.write(struct.pack('<I', 0))
        f.write(struct.pack('<I', width * height * 2))
        f.write(struct.pack('<i', 2835))
        f.write(struct.pack('<i', 2835))
        f.write(struct.pack('<I', 0))
        f.write(struct.pack('<I', 0))
        
        # Pixel data (RGB565)
        for y in range(height - 1, -1, -1):
            for x in range(width):
                r, g, b = img_rgb565.getpixel((x, y))
                rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
                f.write(struct.pack('<H', rgb565))
    
    file_size = 14 + 40 + width * height * 2
    print(f"Wygenerowano mapę pixel art: {filename}")
    print(f"  Rozmiar: {width}x{height}")
    print(f"  Format: BMP 16-bit RGB565")
    print(f"  Rozmiar pliku: {file_size} bajtów")


if __name__ == "__main__":
    # Generuj mapę 140x80 dla QRZ popup (styl pixel art)
    create_pixel_world_map(140, 80, "world_map_pixel_140x80.bmp")
    
    # Generuj mapę 320x180 dla ekranu QRZ Info
    create_pixel_world_map(320, 180, "world_map_pixel_320x180.bmp")
