import java.util.ArrayList;

class ArrayListTest {
    // Create a Main class
    public static class DriveStraight {
        double distance;
        double heading;

        public DriveStraight(double distance, double heading) {
            this.distance = distance;
            this.heading = heading;
        }
    }

    public static class Turn {
        double heading;

        public Turn(double heading) {
            this.heading = heading;
        }
    }

    public static class HoldHeading {
        double heading;
        double holdTime;

        public HoldHeading(double heading, double holdTime) {
            this.heading = heading;
            this.holdTime = holdTime;
        }
    }

    public static void main(String[] args) {
        ArrayList<Object> segments = new ArrayList<>();
        ArrayList<ArrayList<Object>> paths = new ArrayList<>();
        segments.add(new DriveStraight(24, 0));
        segments.add(new Turn(-45));
        segments.add(new HoldHeading(0, .5));
        paths.add(segments);

        segments = new ArrayList<>();
        segments.add(new DriveStraight(17, -45));
        segments.add(new Turn(45));
        segments.add(new HoldHeading(45, .5));
        paths.add(segments);

        segments = new ArrayList<>();
        segments.add(new DriveStraight(17, 45));
        segments.add(new Turn(0));
        segments.add(new HoldHeading(0, 1));
        paths.add(segments);

        segments = new ArrayList<>();
        segments.add(new DriveStraight(-45, 0));
        paths.add(segments);
        for (ArrayList<Object> path : paths) {
            for (Object segment : path) {
                switch (segment.getClass().getSimpleName()) {
                    case "DriveStraight":
                        System.out.println("Drive Straight");
                        break;
                    case "Turn":
                        System.out.println("Turn");
                        break;
                    case "HoldHeading":
                        System.out.println("Hold Heading");
                        break;
                    default:
                        System.out.println("Error");
                }
            }
        }
    }
}
