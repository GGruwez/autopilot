package interfaces;

public class Log {

    public static boolean LOG = false;

    public static void println(Object s){
        if(LOG)
            System.out.println(s.toString());
    }
}
