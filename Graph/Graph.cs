using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Graph
{

    public class GraphEdge<T>
    {
        private GraphNode<T> start_node;
        private GraphNode<T> end_node;
        private double cost;

        public GraphNode<T> Start
        {
            get { return start_node; }
            set { start_node = value; }
        }

        public GraphNode<T> End
        {
            get { return end_node; }
            set { end_node = value; }
        }

        public double Cost
        {
            get { return cost; }
            set { cost = value; }
        }

        public GraphEdge(GraphNode<T> start, GraphNode<T> end, double cost)
        {
            this.start_node = start;
            this.end_node = end;
            this.cost = cost;
        }

        public override string ToString()
        {
            return Start.Value + " -> " + End.Value;
        }
    }

    public class GraphNode<T>
    {
        private T value;
        private LinkedList<GraphEdge<T>> edges;
        private double cost_so_far; //this is the smallest found cost so far
        private GraphEdge<T> connection_edge; //this is the edge that the shortest path so far came from.

        public double CostSoFar
        {
            get { return cost_so_far; }
            set { cost_so_far = value; }
        }

        public GraphEdge<T> Connection
        {
            get { return connection_edge; }
            set { connection_edge = value; }
        }

        public LinkedList<GraphEdge<T>> Links
        {
            get { return edges; }
        }

        public void AddNeighbor(GraphNode<T> n, double cost)
        {
            this.edges.AddLast(new GraphEdge<T>(this, n, cost));
        }
        
        public T Value
        {
            get { return value; }
            set { this.value = value; }
        }

        public GraphNode(T value)
        {
            this.value = value;
            edges = new LinkedList<GraphEdge<T>>();
            cost_so_far = double.MaxValue;
        }

        public override string ToString()
        {
            string s = value.ToString() + " -> ";

            int i = 0;
            foreach(GraphEdge<T> e in edges)
            {
                if (i == edges.Count - 1)
                    s += e.End.Value.ToString();

                else
                    s += e.End.Value.ToString() + ", ";

                i++;
            }

            return s;
        }
    }

    public class Graph<T>
    {
        private LinkedList<GraphNode<T>> nodes;
        private GraphNode<T> start_node;
        private LinkedList<GraphNode<T>> open_nodes;
        private LinkedList<GraphNode<T>> closed_nodes;
        private GraphNode<T> current_node;

        private GraphNode<T> StartNode
        {
            get { return start_node; }
            set { start_node = value; }
        }

        public Graph()
        {
            nodes = new LinkedList<GraphNode<T>>();
            open_nodes = new LinkedList<GraphNode<T>>();
            closed_nodes = new LinkedList<GraphNode<T>>();
        }

        private GraphNode<T> getSmallestCostSoFar()
        {
            double smallest_cost = open_nodes.First.Value.CostSoFar;
            GraphNode<T> smallest_cost_node = open_nodes.First.Value;

            foreach (GraphNode<T> n in open_nodes)
            {
                if (n.CostSoFar < smallest_cost)
                {
                    smallest_cost = n.CostSoFar;
                    smallest_cost_node = n;
                }

            }

            return smallest_cost_node;
        }

        public void Add(GraphNode<T> node)
        {
            nodes.AddLast(node);
        }

        public LinkedList<GraphNode<T>> DijkstraShortestPath(GraphNode<T> start_node, GraphNode<T> end_node)
        {
            this.StartNode = start_node;
            start_node.CostSoFar = 0;
            DijkstraEvaluate();

            GraphNode<T> curr = end_node;
            LinkedList<GraphNode<T>> node_order = new LinkedList<GraphNode<T>>();

            while(curr != start_node)
            {
                node_order.AddFirst(curr);
                curr = curr.Connection.Start;
            }

            node_order.AddFirst(curr);

            return node_order;
        }

        private void DijkstraEvaluate()
        {
            //we evaluate this from the start node
            current_node = start_node;
            open_nodes.AddLast(current_node);

            while(open_nodes.Count > 0)
            {
                DijkstraEvaluateNeihbors();
                if(open_nodes.Count > 0)
                    current_node = getSmallestCostSoFar();
            }

        }

        private void DijkstraEvaluateNeihbors()
        {
            //we look at each of the neighbors of the current node and update their costs so far and connection values
            foreach(GraphEdge<T> e in current_node.Links)
            {
                bool cost_changed = false;
                //for each edge if the cost so far of the start + the cost of the edge we are on is less than the end
                //current cost so far we should update its connection and its cost so far
                if(e.Start.CostSoFar + e.Cost < e.End.CostSoFar)
                {
                    e.End.CostSoFar = e.Start.CostSoFar + e.Cost;
                    e.End.Connection = e;
                    cost_changed = true;
                }

                //add the end node to the open list
                if(!open_nodes.Contains(e.End) && cost_changed)
                    open_nodes.AddLast(e.End);
            }

            open_nodes.Remove(current_node);
            closed_nodes.AddLast(current_node);
            
        }

        public override string ToString()
        {
            string s = string.Empty;
            foreach(GraphNode<T> node in nodes)
            {
                s += node.ToString() + "\n";
            }

            return s;
        }
    }
}
