using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Graph
{
    public class G
    {
        public int i;
        public G(int i)
        {
            this.i = i;
        }

        public override string ToString()
        {
            return i.ToString();
        }
    }

    class Program
    {
        static void Main(string[] args)
        {
            Graph<G> graph = new Graph<G>();

            GraphNode<G> n1 = new GraphNode<G>(new G(5));
            GraphNode<G> n2 = new GraphNode<G>(new G(7));
            GraphNode<G> n3 = new GraphNode<G>(new G(8));
            GraphNode<G> n4 = new GraphNode<G>(new G(9));
            GraphNode<G> n5 = new GraphNode<G>(new G(6));
            GraphNode<G> n6 = new GraphNode<G>(new G(4));
            
            graph.Add(n1);
            graph.Add(n2);
            graph.Add(n3);
            graph.Add(n4);
            graph.Add(n5);
            graph.Add(n6);

            n1.AddNeighbor(n2, 5);
            n2.AddNeighbor(n4, 1);
            n2.AddNeighbor(n3, 7);
            n3.AddNeighbor(n4, 7);
            n3.AddNeighbor(n5, 3);
            n3.AddNeighbor(n6, 6);
            n4.AddNeighbor(n3, 4);
            n4.AddNeighbor(n6, 3);

            Console.WriteLine(graph);

            LinkedList<GraphNode<G>> path = graph.DijkstraShortestPath(n1, n3);

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
