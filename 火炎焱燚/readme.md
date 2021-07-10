# 【十把火】火炎焱燚

## 💡 作品應用主題
火災即時預警＋輔助消防救災 APP

## ⛏️ 選用硬體
- ［ADI］ Smoke Detection
- ［Mouser］ MLX90621 熱成像攝像機
- ［Yutech］ 專用 Pmod 介面轉板
- ［Xilinx］ PYNQ-Z2

## 📝 選料說明
- ［ADI］ Smoke Detection ＋ ［Yutech］ Pmod 轉板
    利用光學式的雙波長量測技術偵測不同的煙霧粒子，助於判斷起火源。

- ［Mouser］ MLX90621 熱成像攝像機
    利用熱紅外線偵測不同位置的溫度，達到精準的熱源辨識。

- ［Xilinx］ PYNQ-Z2
    有多樣擴充接口，且支援影像處理上的硬體加速，適合作為邊緣運算的開發板。

## 🧱 硬體連接架構圖
![](https://i.imgur.com/v3t9GGI.png)

※ 中間為 Arduino Grove Shield，與 PYNQ Shield 可以互通。

![](https://pynq.readthedocs.io/en/v2.5.1/_images/arduino_shield.jpg)

## 👨‍💻 程式碼說明
<pre>
.
├── arduino_grove_MLX90621：MLX90621 microblaze 專案
│   ├── Debug
│   │   ├── arduino_grove_MLX90621.bin
│   │   ├── arduino_grove_MLX90621.elf
│   │   ├── arduino_grove_MLX90621.elf.size
│   │   ├── makefile
│   │   ├── objects.mk
│   │   ├── sources.mk
│   │   └── src
│   │       ├── arduino_grove_MLX90621.d
│   │       ├── arduino_grove_MLX90621.o
│   │       ├── myi2c.d
│   │       ├── myi2c.o
│   │       └── subdir.mk
│   └── src
│       ├── arduino_grove_MLX90621.c：microblaze 主程式碼
│       ├── lscript.ld
│       ├── MLX90621.h
│       ├── myi2c.c：i2c 擴充功能
│       └── myi2c.h
├── arduino_grove_MLX90621.bin
├── arduino_grove_MLX90621.py：MLX90621
├── \_\_init\_\_.py
├── my_iic.py：提供暫存器讀寫功能的 i2c library
└── smoke.py：煙霧偵測 library
</pre>

## ✍️ 團隊成員
<a href="https://github.com/cronus6w6"><img width="100" height="100" src="https://avatars.githubusercontent.com/u/43933375?v=4" title="陳威呈"></img></a> <a href="https://github.com/chihuahuamh"><img width="100" height="100" src="https://avatars.githubusercontent.com/u/40608845?v=4"  title="洪敏嫻"></img></a> <a href="https://github.com/BREND3112317"><img width="100" height="100" src="https://avatars.githubusercontent.com/u/34859596?v=4"  title="林承漢"></img></a> <a href=""><img width="100" height="100" src="https://avatars.githubusercontent.com/u/83815808?v=4"  title="王裕民"></img></a>

## 🙏 致謝
### 人員
- 國立台中科技大學資訊工程系－張家瑋 老師
- 國立台中科技大學語文學院－蒲盈宏 老師
- 中華民國消防設備師（士）協會理事長－吳曉峯 老師
- 建築中心產業育成主任經理－林穎立 老師
- 臺中市政府消防局第二救災救護大隊谷關分隊－張昌龍 老師
- 消防署災害搶救組長－莫懷祖 老師
- 創創 AIOT 主辦單位及工作人員

### 廠商
![](https://i.imgur.com/k2sb6A6.png)![](https://i.imgur.com/BiCBCm0.png)

![](https://i.imgur.com/QUE9yoj.png)![](https://i.imgur.com/s3nVZlT.png)
（以上照字母順序排序）