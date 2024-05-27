import java.util.*;

public class UpdatedAlgo {

    static class Node {
        int id;
        double x, y, z; // Coordinates of the node

        Node(int id, double x, double y, double z) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    static class Edge {
        int from, to;
        double weight;

        Edge(int from, int to, double weight) {
            this.from = from;
            this.to = to;
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
            Node fromNode = getNodeById(from);
            Node toNode = getNodeById(to);

            if (fromNode != null && toNode != null) {
                double calculatedWeight = distance(fromNode, toNode);
                adjList.get(from).add(new Edge(from, to, calculatedWeight));
                adjList.get(to).add(new Edge(to, from, calculatedWeight)); // Assuming undirected graph
            }
        }

        Node getNodeById(int id) {
            for (Node node : nodes) {
                if (node.id == id) {
                    return node;
                }
            }
            return null;
        }

        // Method to calculate the Euclidean distance between two nodes
        static double distance(Node n1, Node n2) {
            return Math.sqrt(Math.pow(n1.x - n2.x, 2) + Math.pow(n1.y - n2.y, 2) + Math.pow(n1.z - n2.z, 2));
        }

        // Method to count the number of nodes within view range of a given position
        static int countNodesInViewRange(double x, double y, double z, List<Node> nodes, double viewRange) {
            int count = 0;
            for (Node node : nodes) {
                if (distance(new Node(-1, x, y, z), node) <= viewRange) {
                    count++;
                }
            }
            return count;
        }

        // Method to find the optimal position for the camera using a grid traversal
        Node findOptimalCameraPosition(double viewRange, double gridStep) {
            double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, minZ = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE, maxY = Double.MIN_VALUE, maxZ = Double.MIN_VALUE;

            // Determine the boundaries of the grid based on the farthest nodes
            for (Node node : nodes) {
                if (node.x < minX) minX = node.x;
                if (node.y < minY) minY = node.y;
                if (node.z < minZ) minZ = node.z;
                if (node.x > maxX) maxX = node.x;
                if (node.y > maxY) maxY = node.y;
                if (node.z > maxZ) maxZ = node.z;
            }

            // Calculate the center of the grid
            double centerX = (minX + maxX) / 2;
            double centerY = (minY + maxY) / 2;
            double centerZ = (minZ + maxZ) / 2;

            Node optimalNode = new Node(-1, centerX, centerY, centerZ);
            int maxCoveredNodes = 0;

            // Traverse each point on the grid
            for (double x = minX; x <= maxX; x += gridStep) {
                for (double y = minY; y <= maxY; y += gridStep) {
                    for (double z = minZ; z <= maxZ; z += gridStep) {
                        int coveredNodes = countNodesInViewRange(x, y, z, nodes, viewRange);
                        // Check if this point has more coverage or if it is closer to the center in case of a tie
                        if (coveredNodes > maxCoveredNodes ||
                            (coveredNodes == maxCoveredNodes && distance(new Node(-1, x, y, z), new Node(-1, centerX, centerY, centerZ)) <
                             distance(optimalNode, new Node(-1, centerX, centerY, centerZ)))) {
                            maxCoveredNodes = coveredNodes;
                            optimalNode = new Node(-1, x, y, z);
                        }
                    }
                }
            }

            return optimalNode;
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
                NodeDistance current = pq.poll();  // Getting 1st node  
                int currentNode = current.node;

                if (currentNode == destination) {
                    break;
                }

                for (Edge edge : adjList.get(currentNode)) {  // Edges from node 1st
                    double newDist = distances.get(currentNode) + edge.weight;  // Previous distance + edge weight
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

            if (path.isEmpty() || path.get(0) != start) {
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

        // Method to convert latitude and longitude to Cartesian coordinates
        static Node latLonToCartesian(int id, double lat, double lon) {
            final double R = 6371; // Earth's radius in km
            double x = R * Math.cos(Math.toRadians(lat)) * Math.cos(Math.toRadians(lon));
            double y = R * Math.cos(Math.toRadians(lat)) * Math.sin(Math.toRadians(lon));
            double z = R * Math.sin(Math.toRadians(lat));
            return new Node(id, x, y, z);
        }

        // Method to convert Cartesian coordinates to latitude and longitude
        static double[] cartesianToLatLon(double x, double y, double z) {
            final double R = 6371; // Earth's radius in km
            double lat = Math.toDegrees(Math.asin(z / R));
            double lon = Math.toDegrees(Math.atan2(y, x));
            return new double[]{lat, lon};
        }
    }

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        Graph graph = new Graph();

        // Adding nodes to the graph (latitude, longitude)
        System.out.println("Enter the number of nodes:");
        int numNodes = scanner.nextInt();
        for (int i = 0; i < numNodes; i++) {
            System.out.println("Enter latitude and longitude for node " + (i + 1) + ":");
            double lat = scanner.nextDouble();
            double lon = scanner.nextDouble();
            graph.addNode(Graph.latLonToCartesian(i + 1, lat, lon));
        }

        // Adding edges to the graph
        System.out.println("Enter the number of edges:");
        int numEdges = scanner.nextInt();
        for (int i = 0; i < numEdges; i++) {
            System.out.println("Enter start node ID and end node ID for edge " + (i + 1) + ":");
            int from = scanner.nextInt();
            int to = scanner.nextInt();
            double weight = Graph.distance(graph.getNodeById(from), graph.getNodeById(to));
            graph.addEdge(from, to, weight);
        }

        System.out.println("Finding optimal camera position...");
        double viewRange = 3.0; // Example view range
        double gridStep = 0.5;  // Example grid step size
        Node optimalNode = graph.findOptimalCameraPosition(viewRange, gridStep);

        if (optimalNode != null) {
            double[] latLon = Graph.cartesianToLatLon(optimalNode.x, optimalNode.y, optimalNode.z);
            System.out.println("Optimal camera position is at coordinates: (" + latLon[0] + ", " + latLon[1] + ")");
        } else {
            System.out.println("No optimal position found.");
        }

        int start, destination;
        do {
            System.out.println("Enter start node ID:");
            start = scanner.nextInt();
            System.out.println("Enter destination node ID:");
            destination = scanner.nextInt();

            if (graph.getNodeById(start) == null || graph.getNodeById(destination) == null) {
                System.out.println("Invalid node ID. Please enter valid node IDs within the range of the graph nodes.");
            }
        } while (graph.getNodeById(start) == null || graph.getNodeById(destination) == null);

        List<Integer> shortestPath = graph.dijkstra(start, destination);

        if (shortestPath.isEmpty()) {
            System.out.println("No path found between node " + start + " and node " + destination);
        } else {
            System.out.println("Shortest path from node " + start + " to node " + destination + " is:");
            for (int nodeId : shortestPath) {
                System.out.print("-->" + nodeId);
            }
        }

        scanner.close();
    }
}
