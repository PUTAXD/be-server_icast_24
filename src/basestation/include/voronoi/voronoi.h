/**
 * @author @danendra10
 * @date 2023-03-19
 */

#include <iostream>
#include <queue>
#include <set>
#include <math.h>

using namespace std;

typedef pair<double, double> point;

#define x first
#define y second

struct arc;
struct seg;

struct event
{
    double x;
    point p;
    arc *a;
    bool valid;

    event(double xx, point pp, arc *aa) : x(xx), p(pp), a(aa), valid(true) {}
};

struct arc
{
    point p;
    arc *prev, *next;
    event *e;

    seg *s0, *s1;

    arc(point pp, arc *a = 0, arc *b = 0) : p(pp), prev(a), next(b), e(0), s0(0), s1(0) {}
};

vector<seg *> output; // Array of output segments

struct seg
{
    point start, end;
    bool done;

    seg(point p) : start(p), end(0, 0), done(false)
    {
        output.push_back(this);
    }

    void finish(point p)
    {
        if (done)
            return;
        end = p;
        done = true;
    }
};

arc *root = 0;

//---> Prototypes
//===============

void ProcessPoints();
void ProcessEvents();
void FrontInsert(point p);

bool Circle(point a, point b, point c, double *x, point *o);
void CheckCircleEvent(arc *i, double x0);

bool Intersect(point p, arc *i, point *result = 0);
point Intersection(point p0, point p1, double l);

void FinishEdges();
void PrintOutput();

void ProcessVoronoiDiagrams();

struct gt
{
    bool operator()(point a, point b) { return a.x == b.x ? a.y > b.y : a.x > b.x; }
    bool operator()(event *a, event *b) { return a->x > b->x; }
};

double X0 = 0, X1 = 0, Y0 = 1200, Y1 = 600;