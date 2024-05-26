import java.util.*;

public class CameraPlacement {

    static class Node {
        int id;
        double x, y; // Coordinates of the node

        Node(int id, double x, double y) {
            this.id = id;
            this.x = x;
            this.y = y;
        }
    }

    static class Graph {
        List<Node> nodes = new ArrayList<>();
        Map<Integer, List<Integer>> adjList = new HashMap<>();

        void addNode(Node node) {
            nodes.add(node);
            adjList.put(node.id, new ArrayList<>());
        }

        void addEdge(int node1, int node2) {
            adjList.get(node1).add(node2);
            adjList.get(node2).add(node1); // the graph is undirected
        }
    }

    // Method to calculate distance between two nodes
    static double distance(Node n1, Node n2) {
        return Math.sqrt(Math.pow(n1.x - n2.x, 2) + Math.pow(n1.y - n2.y, 2));
    }

    // Method to count the number of nodes within view range of a given position
    static int countNodesInViewRange(double x, double y, List<Node> nodes, double viewRange) {
        int count = 0;
        for (Node node : nodes) {
            if (distance(new Node(-1, x, y), node) <= viewRange) {
                count++;
            }
        }
        return count;
    }

    // Method to find the optimal position for the camera using a grid traversal
    static Node findOptimalCameraPosition(Graph graph, double viewRange, double gridStep) {
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE, maxY = Double.MIN_VALUE;

        // Determine the boundaries of the grid based on the farthest nodes X and Y corrdinate
        for (Node node : graph.nodes) {
            if (node.x < minX) minX = node.x;
            if (node.y < minY) minY = node.y;
            if (node.x > maxX) maxX = node.x;
            if (node.y > maxY) maxY = node.y;
        }

         // Calculate the center of the grid
        double centerX = (minX + maxX) / 2;
        double centerY = (minY + maxY) / 2;

        Node optimalNode = null;
        int maxCoveredNodes = 0;

       // Traverse each point on the grid
        for (double x = minX; x <= maxX; x += gridStep) {
            for (double y = minY; y <= maxY; y += gridStep) {
                int coveredNodes = countNodesInViewRange(x, y, graph.nodes, viewRange);
                // Check if this point has more coverage or if it is closer to the center in case of a tie
                if (coveredNodes > maxCoveredNodes ||
                (coveredNodes == maxCoveredNodes && distance(new Node(-1, x, y), new Node(-1, centerX, centerY)) <
                 distance(optimalNode, new Node(-1, centerX, centerY)))) {
                maxCoveredNodes = coveredNodes;
                optimalNode = new Node(-1, x, y);
                }
            }
        }

        return optimalNode;
    }

    public static void main(String[] args) {
        // Example usage:
        Graph graph = new Graph();

        // Adding nodes to the graph
        graph.addNode(new Node(1, 0, 0));
        graph.addNode(new Node(2, 2, 2));
        graph.addNode(new Node(3, 3, 1));
        graph.addNode(new Node(4, 5, 4));
        graph.addNode(new Node(5, 7, 3));

        // Adding edges to the graph
        graph.addEdge(1, 2);
        graph.addEdge(2, 3);
        graph.addEdge(3, 4);
        graph.addEdge(4, 5);

        double viewRange = 3.0; // Example view range
        double gridStep = 0.5;  // Example grid step size

        Node optimalNode = findOptimalCameraPosition(graph, viewRange, gridStep);

        if (optimalNode != null) {
            System.out.println("Optimal camera position is at coordinates: (" + optimalNode.x + ", " + optimalNode.y + ")");
        } else {
            System.out.println("No optimal position found.");
        }
    }
}


