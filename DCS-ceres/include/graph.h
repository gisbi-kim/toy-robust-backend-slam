#ifndef GRAPH_H
#define GRAPH_H

class Node
{
public:
    Node(int index, double x, double y, double theta)
    {
        this->index = index;
        p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = theta;
    }

    int index;
    double *p;
};


class Edge
{
public:
    // Type:
    // 0 : Odometry edge
    // 1 : Loop CLosure Edge
    // 2 : Bogus Edge
    Edge(const Node* a, const Node* b, int edge_type )
    {
        this->a = a;
        this->b = b;
        this->edge_type = edge_type;
    }

    void setEdgePose( double x, double y, double theta )
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    void setInformationMatrix( double I11, double  I12, double  I13, double I22, double I23, double I33 )
    {
        this->I11 = I11;
        this->I12 = I12;
        this->I13 = I13;
        this->I22 = I22;
        this->I23 = I23;
        this->I33 = I33;
    }

    const Node *a, *b;
    double x, y, theta;
    double I11, I12, I13, I22, I23, I33;
    int edge_type;
};


#endif
