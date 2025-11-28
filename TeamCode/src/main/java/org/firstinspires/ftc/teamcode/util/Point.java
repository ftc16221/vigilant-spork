package org.firstinspires.ftc.teamcode.util;

public class Point {
    public int x;
    public Point(int x) {
        this.x = x;
    }
    public static class Two extends Point {
        public int y;
        public Two(int x, int y) {
            super(x);
            this.y = y;
        }
        public static class Three extends Two {
            public int z;
            public Three(int x, int y, int z) {
                super(x, y);
                this.z = z;
            }
    
        }
    }
}
