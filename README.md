# KPP OpenCV 



- **Nama Lengkap** : Sabira Ziraili
- **NRP** : 5054261008

> **Status:** kode sudah berjalan. Pengambilan data kalibrasi dan foto
> dokumentasi menyusul.


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
| `SPASI` | Ambil snapshot saat pola papan catur terdeteksi |
| `C` | Jalankan `cv::calibrateCamera()` dan tulis `yaml/kalibrasi.yaml` |
| `R` | Hapus semua snapshot |
| `Q` / `ESC` | Keluar |

min 15 snapshot

### Menjalankan tracker

```bash
./tracker
```

| Tombol | Fungsi |
|---|---|
| `Q` / `ESC` | Keluar |

belum jalan akrena kalibrasi belum ada

### Tuning warna lewat trackbar

**control_HSL** menyediakan slider interaktif. Pilih objek lewat slider
`Object`(objek aktif ditandai bingkai hijau),
lalu geser `H`, `L`, `S` sampai hanya objek tersebut yang putih bersih.
Slider `Warn Z cm` mengatur ambang jarak peringatan.






## 2. Analisis  & Kendala

**Kendala 1 — Merah ada di dua ujung Hue.** 54% piksel botol di H 0–15, 22% di
H 165–179. `cv::inRange` butuh min < max jadi satu rentang tidak bisa menangkap
keduanya. Dipakai dua pita digabung `cv::bitwise_or` lewat subclass
`WrappedHueObject`.

**Kendala 2 — Helm neon tidak jreng.** `S Min` 120 bikin helm tidak terdeteksi;
saturasi aslinya cuma 88–126 karena auto-exposure memucatkan objek terang.
Diturunkan ke 60.

**Kendala 3 — Morfologi menggeser lebar box.** Erosi dan dilasi tidak seimbang
bikin box menyusut, dan Z ikut melenceng karena lebar box dipakai sebagai
`w_pixel`. Disamakan jadi 2 + 2 berbanding 4.

**Kendala 4 — Helm dan pouch tidak simetris.** Diputar sedikit, `w_pixel`
berubah dan Z geser padahal jaraknya tetap. Harus selalu menghadap kamera pada
orientasi yang sama.

**Kendala 5 — Pencahayaan.** Rentang HLS beda tiap ruangan, jadi semua ambang
dipasang di trackbar supaya bisa ditala tanpa compile ulang.
