import java.util.*;

public class ShortestPath {

    static class Node {
        int id;
        double x, y; // Coordinates of the node

        Node(int id, double x, double y) {
            this.id = id;
            this.x = x;
            this.y = y;
        }
    }

    static class Edge {
        int from, to;
        double weight;

        Edge(int from, int to, double weight) {
            this.from = from;
            this.to = to;
            this.weight = weight;
        }
    }

    static class Graph {
        List<Node> nodes = new ArrayList<>();
        Map<Integer, List<Edge>> adjList = new HashMap<>();

        void addNode(Node node) {
            nodes.add(node);
            adjList.put(node.id, new ArrayList<>());
        }

        void addEdge(int from, int to, double weight) {
            adjList.get(from).add(new Edge(from, to, weight));
            adjList.get(to).add(new Edge(to, from, weight)); // Assuming undirected graph
        }

        // Dijkstra's algorithm to find the shortest path
        List<Integer> dijkstra(int start, int destination) {
            Map<Integer, Double> distances = new HashMap<>();
            Map<Integer, Integer> previous = new HashMap<>();
            PriorityQueue<NodeDistance> pq = new PriorityQueue<>(Comparator.comparingDouble(nd -> nd.distance));
            
            for (Node node : nodes) {
                distances.put(node.id, Double.MAX_VALUE);
                previous.put(node.id, null);
            }

            distances.put(start, 0.0);
            pq.add(new NodeDistance(start, 0.0));

            while (!pq.isEmpty()) {
                NodeDistance current = pq.poll();
                int currentNode = current.node;

                if (currentNode == destination) {
                    break;
                }

                for (Edge edge : adjList.get(currentNode)) {
                    double newDist = distances.get(currentNode) + edge.weight;
                    if (newDist < distances.get(edge.to)) {
                        distances.put(edge.to, newDist);
                        previous.put(edge.to, currentNode);
                        pq.add(new NodeDistance(edge.to, newDist));
                    }
                }
            }

            // Reconstruct the path
            List<Integer> path = new ArrayList<>();
            for (Integer at = destination; at != null; at = previous.get(at)) {
                path.add(at);
            }
            Collections.reverse(path);

            if (path.get(0) != start) {
                return Collections.emptyList(); // No path found
            }
            return path;
        }

        static class NodeDistance {
            int node;
            double distance;

            NodeDistance(int node, double distance) {
                this.node = node;
                this.distance = distance;
            }
        }
    }

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        Graph graph = new Graph();

        // Adding nodes to the graph
        graph.addNode(new Node(1, 0, 0));
        graph.addNode(new Node(2, 2, 2));
        graph.addNode(new Node(3, 3, 1));
        graph.addNode(new Node(4, 5, 4));
        graph.addNode(new Node(5, 7, 3));

        // Adding edges to the graph
        graph.addEdge(1, 2, 2.0);
        graph.addEdge(2, 3, 1.0);
        graph.addEdge(3, 4, 2.0);
        graph.addEdge(4, 5, 3.0);
        graph.addEdge(1, 3, 2.5);
        graph.addEdge(2, 4, 2.2);
        graph.addEdge(3, 5, 2.8);

        // Taking user input for start and destination nodes
        System.out.println("Enter start node ID:");
        int start = scanner.nextInt();
        System.out.println("Enter destination node ID:");
        int destination = scanner.nextInt();

        // Finding the shortest path
        List<Integer> shortestPath = graph.dijkstra(start, destination);

        if (shortestPath.isEmpty()) {
            System.out.println("No path found between node " + start + " and node " + destination);
        } else {
            System.out.println("Shortest path from node " + start + " to node " + destination + " is:");
            for (int nodeId : shortestPath) {
                System.out.print(nodeId + " ");
            }
        }

        scanner.close();
    }
}
