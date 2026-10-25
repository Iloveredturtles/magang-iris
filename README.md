# KPP OpenCV 



- **Nama Lengkap** : Sabira Ziraili
- **NRP** : 5054261008


## 1. Cara Run


Build dari root project:

```bash
g++ src/calib.cpp -o calib $(pkg-config --cflags --libs opencv4)
g++ src/main.cpp -Iinclude -o tracker $(pkg-config --cflags --libs opencv4)
```


### Kalibrasi kamera

```bash
./calib
```

| Tombol | Fungsi |
|---|---|
| `SPASI` | Capture gambar |
| `C` | Jalankan kalibrasi, disave ke `yaml/kalibrasi.yaml` |
| `R` | Ulang ambil gambar dari 0 |
| `Q` / `ESC` | Keluar |

min 15 snapshot

### Menjalankan tracker

```bash
./tracker
```

| Tombol | Fungsi |
|---|---|
| `SPASI` | Capture dan menyimpan ke `docs/` |
| `Q` / `ESC` | Keluar |

belum jalan akrena kalibrasi belum ada

### Tuning warna lewat trackbar

**control_HSL** menyediakan slider interaktif. Pilih objek lewat slider
`Object`(objek aktif ditandai bingkai hijau),
lalu geser `H`, `L`, `S` sampai hanya objek tersebut yang putih bersih.
Slider `Warn Z cm` mengatur ambang jarak peringatan.


## 2. Struktur Class

| Class | Peran |
|---|---|
| `CameraIntrinsics` | Baca fx, fy, cx, cy dari YAML |
| `TrackedObject` | **Base class** abstrak, `buildMask()` pure virtual |
| `SolidColorObject` | **Subclass** satu rentang Hue |
| `GradientColorObject` | **Subclass** dua rentang Hue digabung `cv::bitwise_or` |

`buildMask()` dibuat virtual, jadi loop utama cukup panggil
`object->detect(...)` tanpa tahu objek mana yang butuh perlakuan khusus.


## 3. Analisis  & Kendala

**Kendala 1 — beberapa benda memiliki warna di dua ujung Hue.** 

**Kendala 2 — Pencahayaan.** Rentang HLS beda tiap ruangan, jadi semua ambang
dipasang di trackbar supaya bisa ditala tanpa compile ulang.