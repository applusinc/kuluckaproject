
#include <LiquidCrystal_I2C.h>
#include <cstring> // strlen fonksiyonu için

const int LCD_COLUMNS = 20;
const int LCD_ROWS = 4;

class LcdUtil {
    public:
        LiquidCrystal_I2C* plcd = nullptr;
        void begin(LiquidCrystal_I2C &lcd) {
            plcd = &lcd;
        }
        void printCentered(const char *text, int row) {
            if (plcd == nullptr) {
            // LCD nesnesi başlatılmamışsa hiçbir şey yapma
            return;
        }

        if (row < 0 || row >= LCD_ROWS) {
            // Satır geçersizse hiçbir şey yapma
            return;
        }
        plcd->setCursor(0,row);
        plcd->print("                    "); // Satıra padding ekleme

        int textLength = strlen(text);
        int padding = (LCD_COLUMNS - textLength) / 2;
        
        plcd->setCursor(padding, row);
        plcd->print(text);
    }

    void printCentered(String *text, int row) {
            if (plcd == nullptr) {
            // LCD nesnesi başlatılmamışsa hiçbir şey yapma
            return;
        }

        if (row < 0 || row >= LCD_ROWS) {
            // Satır geçersizse hiçbir şey yapma
            return;
        }
        plcd->setCursor(0,row);
        plcd->print("                    "); // Satıra padding ekleme

        int textLength = strlen(text->c_str());
        int padding = (LCD_COLUMNS - textLength) / 2;
        
        plcd->setCursor(padding, row);
        plcd->print(text->c_str());
    }

    
};
