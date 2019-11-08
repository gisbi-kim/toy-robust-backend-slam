#ifndef G2O_UTIL_H
#define G2O_UTIL_H

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include "graph.h"

#define ODOMETRY_EDGE 0
#define CLOSURE_EDGE 1
#define BOGUS_EDGE 2

using namespace std;

class ReadG2O
{
public:
    ReadG2O(const string& fName)
    {
      // Read the file in g2o format
        fstream fp;
        fp.open(fName.c_str(), ios::in);


        string line;
        int v = 0;
        int e = 0;
        while( std::getline(fp, line) )
        {
            vector<string> words;
            boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
            if( words[0].compare( "VERTEX_SE2") == 0 || words[0].compare( "VERTEX2") == 0)
            {
                v++;
                int node_index = boost::lexical_cast<int>( words[1] );
                double x = boost::lexical_cast<double>( words[2] );
                double y = boost::lexical_cast<double>( words[3] );
                double theta = boost::lexical_cast<double>( words[4] );

                Node * node = new Node(node_index, x, y, theta);
                nNodes.push_back( node );
            }


            if( words[0].compare( "EDGE_SE2") == 0 || words[0].compare( "EDGE2") == 0 )
            {
              // cout << e << words[0] << endl;
                int a_indx = boost::lexical_cast<int>( words[1] );
                int b_indx = boost::lexical_cast<int>( words[2] );

                double dx = boost::lexical_cast<double>( words[3] );
                double dy = boost::lexical_cast<double>( words[4] );
                double dtheta = boost::lexical_cast<double>( words[5] );

                double I11, I12, I13, I22, I23, I33;
                I11 = boost::lexical_cast<double>( words[6] );
                I12 = boost::lexical_cast<double>( words[7] );
                I13 = boost::lexical_cast<double>( words[8] );
                I22 = boost::lexical_cast<double>( words[9] );
                I23 = boost::lexical_cast<double>( words[10] );
                I33 = boost::lexical_cast<double>( words[11] );

                if( abs(a_indx - b_indx) < 5 )
                {
                  Edge * edge = new Edge( nNodes[a_indx], nNodes[b_indx], ODOMETRY_EDGE );
                  edge->setEdgePose(dx, dy, dtheta);
                  edge->setInformationMatrix(I11, I12, I13, I22, I23, I33);
                  nEdgesOdometry.push_back(edge);
                }
                else
                {
                  Edge * edge = new Edge( nNodes[a_indx], nNodes[b_indx], CLOSURE_EDGE );
                  edge->setEdgePose(dx, dy, dtheta);
                  edge->setInformationMatrix(I11, I12, I13, I22, I23, I33);
                  nEdgesClosure.push_back(edge);
                }


                e++;
            }

        }

    }


    // write nodes to file to be visualized with python script
    void writePoseGraph_nodes( const string& fname )
    {
      cout << "writePoseGraph nodes: " << fname << endl;
      fstream fp;
      fp.open( fname.c_str(), ios::out );
      for( int i=0 ; i<this->nNodes.size() ; i++ )
      {
        fp << nNodes[i]->index << " " << nNodes[i]->p[0] << " " << nNodes[i]->p[1] << " " << nNodes[i]->p[2]  << endl;
      }
    }

    void writePoseGraph_edges( const string& fname )
    {
      cout << "writePoseGraph Edges : "<< fname << endl;
      fstream fp;
      fp.open( fname.c_str(), ios::out );
      write_edges( fp, this->nEdgesOdometry );
      write_edges( fp, this->nEdgesClosure );
      write_edges( fp, this->nEdgesBogus );
    }

    void writePoseGraph_switches( const string& fname, vector<double>& priors, vector<double*>& optimized )
    {
        cout << "#Closure Edges : "<< nEdgesClosure.size() << endl;
        cout << "#Bogus Edges : "<< nEdgesBogus.size()<< endl;
        cout << "#priors : "<< priors.size()<< endl;
        cout << "#optimized " << optimized.size()<< endl;
        fstream fp;
        fp.open( fname.c_str(), ios::out );
        fp << "Odometry EDGES AHEAD\n";
        for( int i=0 ; i<nEdgesOdometry.size() ; i++ )
        {
            Edge * ed = nEdgesOdometry[i];
            fp << ed->a->index << " " << ed->b->index << " " << ed->edge_type <<
                    " " << 1.0 << " " << 1.0 << endl;
        }


        fp << "Closure EDGES AHEAD\n";
        for( int i=0 ; i<nEdgesClosure.size() ; i++ )
        {
            Edge * ed = nEdgesClosure[i];
            fp << ed->a->index << " " << ed->b->index << " " << ed->edge_type <<
                    " " << priors[i] << " " << *(optimized[i]) << endl;
        }

        fp << "BOGUS EDGES AHEAD\n";
        int ofset = nEdgesClosure.size();
        for( int i=0 ; i<nEdgesBogus.size() ; i++ )
        {
            Edge * ed = nEdgesBogus[i];
            fp << ed->a->index << " " << ed->b->index << " " << ed->edge_type <<
                    " " << priors[ofset+i] << " " << *(optimized[ofset+i]) << endl;
        }

    }

    // Adding Bogus edges as described in Vertigo paper
    void add_random_C(int count )
    {
        int MIN = 0 ;
        int MAX = nNodes.size();
        cout << "Adding Bogus edges as described in Vertigo paper" << endl;
        for( int i = 0 ; i<count ; i++ )
        {

            int a = rand() % MAX;
            int b = rand() % MAX;
            cout << "  " << a << "<--->" << b << endl;
            Edge * edge = new Edge( nNodes[a], nNodes[b], BOGUS_EDGE );
            edge->setEdgePose( rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX );
            nEdgesBogus.push_back( edge );
        }
    }

//private:
    vector<Node*> nNodes; //storage for node
    vector<Edge*> nEdgesOdometry; //storage for edges - odometry
    vector<Edge*> nEdgesClosure; //storage for edges - odometry
    vector<Edge*> nEdgesBogus; //storage for edges - odometry

    void write_edges( fstream& fp, vector<Edge*>& vec )
    {
      for( int i=0 ; i<vec.size() ; i++ )
      {
        // fp << nEdges[i]->a->index << " " << nEdges[i]->b->index << " " << (nEdges[i]->bogus_edge?1:0)  << " " << nEdges[i]->switch_var[0]<< endl;
        fp << vec[i]->a->index << " " << vec[i]->b->index << " " << vec[i]->edge_type << endl;
      }
    }

};

#endif
