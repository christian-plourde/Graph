using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Graph
{
    public class G : IHeuristic<G>
    {
        public string name;
        public G(string name)
        {
            this.name = name;
        }

        public override string ToString()
        {
            return name.ToString();
        }

        public double ComputeHeuristic(G goal)
        {
            return 5;
        }
    }

    class Program
    {
        static void Main(string[] args)
        {
            PathFinderGraph<G> graph = new PathFinderGraph<G>();
            //Graph<G> graph = new Graph<G>();

            GraphNode<G> ns = new GraphNode<G>(new G("S"));
            GraphNode<G> na = new GraphNode<G>(new G("A"));
            GraphNode<G> nb = new GraphNode<G>(new G("B"));
            GraphNode<G> nc = new GraphNode<G>(new G("C"));
            GraphNode<G> nd = new GraphNode<G>(new G("D"));
            GraphNode<G> ne = new GraphNode<G>(new G("E"));
            GraphNode<G> ng = new GraphNode<G>(new G("G"));
            
            graph.Add(ns);
            graph.Add(na);
            graph.Add(nb);
            graph.Add(nc);
            graph.Add(nd);
            graph.Add(ne);
            graph.Add(ng);

            ns.AddNeighbor(na, 3);
            ns.AddNeighbor(nb, 10);
            na.AddNeighbor(nb, 5);
            na.AddNeighbor(nc, 9);
            na.AddNeighbor(nd, 6);
            nb.AddNeighbor(nc, 3);
            nb.AddNeighbor(ng, 15);
            nc.AddNeighbor(nd, 9);
            nc.AddNeighbor(ne, 7);
            nc.AddNeighbor(ng, 6);
            nd.AddNeighbor(ne, 6);
            ne.AddNeighbor(ng, 3);

            Console.WriteLine(graph);

            LinkedList<GraphNode<G>> path = graph.ShortestPath(ns, ng);

            string s = string.Empty;
            int i = 0;

            foreach(GraphNode<G> n in path)
            {
                if (i == path.Count - 1)
                    s += n.Value;
                else
                    s += n.Value + " -> ";
                i++;
            }

            Console.WriteLine(s);

            Console.ReadLine();

        }
    }
}
