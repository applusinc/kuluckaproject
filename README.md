
# Kuluçka Makinesi Yazılımı

Projenin amacı Tarım ve Orman bakanlığının Kuluçka Makineleri Deney İlkelerine uyarak tavuk yumurtalarından verimli döl ve civciv çıkarmaktır.



## Özellikler
Makine, Sıcaklık, nem ve oksijen seviyesini ideal koşullarda tutabilmektedir.
LCD Ekran ve uyarı ve bilgi ledleri ile sizi anlık olarak bilgilendirmektedir.
Yumurtanın içindeki hücrenin yer çekiminin etkisiyle kabuğa yapışmaması için düzenli olarak step motor ile yumurtalar otomatik döndürülmektedir.
Sıcaklığa ve neme göre resistanca röle ile AC ile güç verip dengeyi sağlamaktadır.
Hava akışını sağlamak için bir adet kontrol edilebilir fan ve oksijen girebilmesi için bir adet kontrol edilebilir egzoz fanı bulunmaktadır.
İçerisinde nemi sağlayan su miktarını ölçüp sizi uyarabilir.
18. günden sonra yumurtaların değişen ortam koşullarını ihtiyacını makine otomatik olarak gidermektedir.
Anlık olarak Sıcaklık ve Nem Oranını lcd ekrandan kontrol edebilir. Gün Saat Dakika Olarak yumurtaları koyduğunuz günden uzaklığını hesaplar.
Yüksek Doğruluklu ve stabil DHT22 sensörü ile sıcaklığı ve nemi anlık olarak ölçebilir.
Evinizde ki Wi-Fi ağına otomatik bağlanıp içerisinde ki konfigrasyon verilerini okuyabilir değiştirebilirsiniz.
## Proje Mimarisi

Projem ESP32 ile C++ dilinde Platform IO kullanılarak yazılmıştır. 
  
## Bağımlı Kütüphaneler

- ArduinoJson
- ESPAsyncWebServer
- LiquidCrystal_I2C_ESP32
- Preferences
- Time
- DHT Sensor Lib
- Async TCP
- Stepper

  
