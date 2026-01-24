/**
 * P. Rutins 2026 - Manchester Machine Makers FTC #16221
 * Altered from https://gist.github.com/nightlark/6482130?permalink_comment_id=1609353
 */

package org.firstinspires.ftc.teamcode.util;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;
import java.util.stream.Stream;

public class ColorNameLookup {
    public static class ColorName {
        protected int r,g,b;
        protected String name;

        public ColorName(String name, int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.name = name;
        }

        public int computeMSE(int pixR, int pixG, int pixB) {
            return (((pixR-r)*(pixR-r) + (pixG-g)*(pixG-g) + (pixB-b)*(pixB-b))/3);
        }

        public int getIntValue() {
            return android.graphics.Color.rgb(r, g, b);
        }

        public int getR() {
            return r;
        }

        public int getG() {
            return g;
        }

        public int getB() {
            return b;
        }

        public String getName() {
            return name;
        }
    }

    public static ArrayList<ColorName> colorList = new ArrayList<>(Arrays.asList(new ColorName[] {
            new ColorName("AliceBlue", 0xF0, 0xF8, 0xFF),
            new ColorName("AntiqueWhite", 0xFA, 0xEB, 0xD7),
            new ColorName("Aqua", 0x00, 0xFF, 0xFF),
            new ColorName("Aquamarine", 0x7F, 0xFF, 0xD4),
            new ColorName("Azure", 0xF0, 0xFF, 0xFF),
            new ColorName("Beige", 0xF5, 0xF5, 0xDC),
            new ColorName("Bisque", 0xFF, 0xE4, 0xC4),
            new ColorName("Black", 0x00, 0x00, 0x00),
            new ColorName("BlanchedAlmond", 0xFF, 0xEB, 0xCD),
            new ColorName("Blue", 0x00, 0x00, 0xFF),
            new ColorName("BlueViolet", 0x8A, 0x2B, 0xE2),
            new ColorName("Brown", 0xA5, 0x2A, 0x2A),
            new ColorName("BurlyWood", 0xDE, 0xB8, 0x87),
            new ColorName("CadetBlue", 0x5F, 0x9E, 0xA0),
            new ColorName("Chartreuse", 0x7F, 0xFF, 0x00),
            new ColorName("Chocolate", 0xD2, 0x69, 0x1E),
            new ColorName("Coral", 0xFF, 0x7F, 0x50),
            new ColorName("CornflowerBlue", 0x64, 0x95, 0xED),
            new ColorName("Cornsilk", 0xFF, 0xF8, 0xDC),
            new ColorName("Crimson", 0xDC, 0x14, 0x3C),
            new ColorName("Cyan", 0x00, 0xFF, 0xFF),
            new ColorName("DarkBlue", 0x00, 0x00, 0x8B),
            new ColorName("DarkCyan", 0x00, 0x8B, 0x8B),
            new ColorName("DarkGoldenRod", 0xB8, 0x86, 0x0B),
            new ColorName("DarkGray", 0xA9, 0xA9, 0xA9),
            new ColorName("DarkGreen", 0x00, 0x64, 0x00),
            new ColorName("DarkKhaki", 0xBD, 0xB7, 0x6B),
            new ColorName("DarkMagenta", 0x8B, 0x00, 0x8B),
            new ColorName("DarkOliveGreen", 0x55, 0x6B, 0x2F),
            new ColorName("DarkOrange", 0xFF, 0x8C, 0x00),
            new ColorName("DarkOrchid", 0x99, 0x32, 0xCC),
            new ColorName("DarkRed", 0x8B, 0x00, 0x00),
            new ColorName("DarkSalmon", 0xE9, 0x96, 0x7A),
            new ColorName("DarkSeaGreen", 0x8F, 0xBC, 0x8F),
            new ColorName("DarkSlateBlue", 0x48, 0x3D, 0x8B),
            new ColorName("DarkSlateGray", 0x2F, 0x4F, 0x4F),
            new ColorName("DarkTurquoise", 0x00, 0xCE, 0xD1),
            new ColorName("DarkViolet", 0x94, 0x00, 0xD3),
            new ColorName("DeepPink", 0xFF, 0x14, 0x93),
            new ColorName("DeepSkyBlue", 0x00, 0xBF, 0xFF),
            new ColorName("DimGray", 0x69, 0x69, 0x69),
            new ColorName("DodgerBlue", 0x1E, 0x90, 0xFF),
            new ColorName("FireBrick", 0xB2, 0x22, 0x22),
            new ColorName("FloralWhite", 0xFF, 0xFA, 0xF0),
            new ColorName("ForestGreen", 0x22, 0x8B, 0x22),
            new ColorName("Fuchsia", 0xFF, 0x00, 0xFF),
            new ColorName("Gainsboro", 0xDC, 0xDC, 0xDC),
            new ColorName("GhostWhite", 0xF8, 0xF8, 0xFF),
            new ColorName("Gold", 0xFF, 0xD7, 0x00),
            new ColorName("GoldenRod", 0xDA, 0xA5, 0x20),
            new ColorName("Gray", 0x80, 0x80, 0x80),
            new ColorName("Green", 0x00, 0x80, 0x00),
            new ColorName("GreenYellow", 0xAD, 0xFF, 0x2F),
            new ColorName("HoneyDew", 0xF0, 0xFF, 0xF0),
            new ColorName("HotPink", 0xFF, 0x69, 0xB4),
            new ColorName("IndianRed", 0xCD, 0x5C, 0x5C),
            new ColorName("Indigo", 0x4B, 0x00, 0x82),
            new ColorName("Ivory", 0xFF, 0xFF, 0xF0),
            new ColorName("Khaki", 0xF0, 0xE6, 0x8C),
            new ColorName("Lavender", 0xE6, 0xE6, 0xFA),
            new ColorName("LavenderBlush", 0xFF, 0xF0, 0xF5),
            new ColorName("LawnGreen", 0x7C, 0xFC, 0x00),
            new ColorName("LemonChiffon", 0xFF, 0xFA, 0xCD),
            new ColorName("LightBlue", 0xAD, 0xD8, 0xE6),
            new ColorName("LightCoral", 0xF0, 0x80, 0x80),
            new ColorName("LightCyan", 0xE0, 0xFF, 0xFF),
            new ColorName("LightGoldenRodYellow", 0xFA, 0xFA, 0xD2),
            new ColorName("LightGray", 0xD3, 0xD3, 0xD3),
            new ColorName("LightGreen", 0x90, 0xEE, 0x90),
            new ColorName("LightPink", 0xFF, 0xB6, 0xC1),
            new ColorName("LightSalmon", 0xFF, 0xA0, 0x7A),
            new ColorName("LightSeaGreen", 0x20, 0xB2, 0xAA),
            new ColorName("LightSkyBlue", 0x87, 0xCE, 0xFA),
            new ColorName("LightSlateGray", 0x77, 0x88, 0x99),
            new ColorName("LightSteelBlue", 0xB0, 0xC4, 0xDE),
            new ColorName("LightYellow", 0xFF, 0xFF, 0xE0),
            new ColorName("Lime", 0x00, 0xFF, 0x00),
            new ColorName("LimeGreen", 0x32, 0xCD, 0x32),
            new ColorName("Linen", 0xFA, 0xF0, 0xE6),
            new ColorName("Magenta", 0xFF, 0x00, 0xFF),
            new ColorName("Maroon", 0x80, 0x00, 0x00),
            new ColorName("MediumAquaMarine", 0x66, 0xCD, 0xAA),
            new ColorName("MediumBlue", 0x00, 0x00, 0xCD),
            new ColorName("MediumOrchid", 0xBA, 0x55, 0xD3),
            new ColorName("MediumPurple", 0x93, 0x70, 0xDB),
            new ColorName("MediumSeaGreen", 0x3C, 0xB3, 0x71),
            new ColorName("MediumSlateBlue", 0x7B, 0x68, 0xEE),
            new ColorName("MediumSpringGreen", 0x00, 0xFA, 0x9A),
            new ColorName("MediumTurquoise", 0x48, 0xD1, 0xCC),
            new ColorName("MediumVioletRed", 0xC7, 0x15, 0x85),
            new ColorName("MidnightBlue", 0x19, 0x19, 0x70),
            new ColorName("MintCream", 0xF5, 0xFF, 0xFA),
            new ColorName("MistyRose", 0xFF, 0xE4, 0xE1),
            new ColorName("Moccasin", 0xFF, 0xE4, 0xB5),
            new ColorName("NavajoWhite", 0xFF, 0xDE, 0xAD),
            new ColorName("Navy", 0x00, 0x00, 0x80),
            new ColorName("OldLace", 0xFD, 0xF5, 0xE6),
            new ColorName("Olive", 0x80, 0x80, 0x00),
            new ColorName("OliveDrab", 0x6B, 0x8E, 0x23),
            new ColorName("Orange", 0xFF, 0xA5, 0x00),
            new ColorName("OrangeRed", 0xFF, 0x45, 0x00),
            new ColorName("Orchid", 0xDA, 0x70, 0xD6),
            new ColorName("PaleGoldenRod", 0xEE, 0xE8, 0xAA),
            new ColorName("PaleGreen", 0x98, 0xFB, 0x98),
            new ColorName("PaleTurquoise", 0xAF, 0xEE, 0xEE),
            new ColorName("PaleVioletRed", 0xDB, 0x70, 0x93),
            new ColorName("PapayaWhip", 0xFF, 0xEF, 0xD5),
            new ColorName("PeachPuff", 0xFF, 0xDA, 0xB9),
            new ColorName("Peru", 0xCD, 0x85, 0x3F),
            new ColorName("Pink", 0xFF, 0xC0, 0xCB),
            new ColorName("Plum", 0xDD, 0xA0, 0xDD),
            new ColorName("PowderBlue", 0xB0, 0xE0, 0xE6),
            new ColorName("Purple", 0x80, 0x00, 0x80),
            new ColorName("Red", 0xFF, 0x00, 0x00),
            new ColorName("RosyBrown", 0xBC, 0x8F, 0x8F),
            new ColorName("RoyalBlue", 0x41, 0x69, 0xE1),
            new ColorName("SaddleBrown", 0x8B, 0x45, 0x13),
            new ColorName("Salmon", 0xFA, 0x80, 0x72),
            new ColorName("SandyBrown", 0xF4, 0xA4, 0x60),
            new ColorName("SeaGreen", 0x2E, 0x8B, 0x57),
            new ColorName("SeaShell", 0xFF, 0xF5, 0xEE),
            new ColorName("Sienna", 0xA0, 0x52, 0x2D),
            new ColorName("Silver", 0xC0, 0xC0, 0xC0),
            new ColorName("SkyBlue", 0x87, 0xCE, 0xEB),
            new ColorName("SlateBlue", 0x6A, 0x5A, 0xCD),
            new ColorName("SlateGray", 0x70, 0x80, 0x90),
            new ColorName("Snow", 0xFF, 0xFA, 0xFA),
            new ColorName("SpringGreen", 0x00, 0xFF, 0x7F),
            new ColorName("SteelBlue", 0x46, 0x82, 0xB4),
            new ColorName("Tan", 0xD2, 0xB4, 0x8C),
            new ColorName("Teal", 0x00, 0x80, 0x80),
            new ColorName("Thistle", 0xD8, 0xBF, 0xD8),
            new ColorName("Tomato", 0xFF, 0x63, 0x47),
            new ColorName("Turquoise", 0x40, 0xE0, 0xD0),
            new ColorName("Violet", 0xEE, 0x82, 0xEE),
            new ColorName("Wheat", 0xF5, 0xDE, 0xB3),
            new ColorName("White", 0xFF, 0xFF, 0xFF),
            new ColorName("WhiteSmoke", 0xF5, 0xF5, 0xF5),
            new ColorName("Yellow", 0xFF, 0xFF, 0x00),
            new ColorName("YellowGreen", 0x9A, 0xCD, 0x32)
    }));

    public static ColorName getColorByInt(int color) {
        ColorName foundColor = (ColorName) colorList.stream()
                .filter(c -> (Objects.equals(c.getIntValue(), color))).toArray()[0];
        if (foundColor != null) {
            return foundColor;
        }
        // couldn't just find it, try a closest match
        int minMSE = Integer.MAX_VALUE;
        int mse;
        for (ColorName c : colorList) {
            mse = color;
            if (mse < minMSE) {
                minMSE = mse;
                foundColor = c;
            }
        }
        return foundColor;
    }

    public static ColorName getColorByRGB(int r, int g, int b) {
        return getColorByInt(android.graphics.Color.rgb(r, g, b));
    }

    public static ColorName getColorByName(String colorName) {
        // case-insensitive match
        Stream<ColorName> color = colorList.stream()
                .filter(c -> (Objects.equals(c.getName().toUpperCase(), colorName.toUpperCase())));
        ColorName[] cArray = color.toArray(ColorName[]::new);
        return cArray[0];
    }
}
