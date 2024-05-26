import java.util.*;

public class OptimalCameraPlacement {
    static class Node {
        double x, y;

        public Node(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public static double distance(Node n1, Node n2) {
        return Math.sqrt((n1.x - n2.x) * (n1.x - n2.x) + (n1.y - n2.y) * (n1.y - n2.y));
    }

    public static int countNodesWithinRange(Node camera, List<Node> nodes, double viewRange) {
        int count = 0;
        for (Node node : nodes) {
            if (distance(camera, node) <= viewRange) {
                count++;
            }
        }
        return count;
    }

    public static Node findOptimalCameraPosition(List<Node> nodes, double viewRange) {
        Node optimalPosition = null;
        int maxCount = 0;

        for (double x = 0; x <= 1000; x += 1) { // Adjust based on your coordinates range
            for (double y = 0; y <= 1000; y += 1) { // Adjust based on your coordinates range
                Node cameraPosition = new Node(x, y);
                int count = countNodesWithinRange(cameraPosition, nodes, viewRange);
                if (count > maxCount) {
                    maxCount = count;
                    optimalPosition = cameraPosition;
                }
            }
        }

        return optimalPosition;
    }

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        
        System.out.println("Enter the number of nodes:");
        int n = scanner.nextInt();
        List<Node> nodes = new ArrayList<>();
        
        System.out.println("Enter the coordinates of the nodes (x y):");
        for (int i = 0; i < n; i++) {
            double x = scanner.nextDouble();
            double y = scanner.nextDouble();
            nodes.add(new Node(x, y));
        }
        
        System.out.println("Enter the view range of the camera:");
        double viewRange = scanner.nextDouble();
        
        Node optimalPosition = findOptimalCameraPosition(nodes, viewRange);
        System.out.println("Optimal position to place the camera: (" + optimalPosition.x + ", " + optimalPosition.y + ")");
        
        scanner.close();
    }
}
