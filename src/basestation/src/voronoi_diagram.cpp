/**
 * @author @danendra10
 * @date 2023-03-19
 * @brief This file contains the implementation of the Voronoi Diagrams
 * @ref https://en.wikipedia.org/wiki/Voronoi_diagram
 */
#include "voronoi/voronoi.h"

priority_queue<point, vector<point>, gt> points;
priority_queue<event, vector<event *>, gt> events;

uint8_t isRobotReady(uint8_t index_robot)
{
    uint8_t is_robot_ready = 0;
    if (voronoi_is_ready[index_robot] && voronoi_status_control_robot[index_robot])
    {
        is_robot_ready = 1;
    }
    return is_robot_ready;
}

void ProcessVoronoiDiagrams()
{
    for (uint8_t i = 0; i < 5; i++)
    {
        if (isRobotReady(i))
        {
            points.push(point(voronoi_robot_pos_x[i], voronoi_robot_pos_y[i]));
        }
    }

    // Add margins to the bounding box.
    double dx = (X1 - X0 + 1) / 5.0;
    double dy = (Y1 - Y0 + 1) / 5.0;

    // put the margins
    X0 -= dx;
    X1 += dx;
    Y0 -= dy;
    Y1 += dy;
    if (points.empty())
        return;
    // Process the queues; select the top element with smaller x coordinate.
    while (!points.empty())
        if (!events.empty() && events.top()->x <= points.top().x)
            ProcessEvents();
        else
            ProcessPoints();
    // After all points are processed, do the remaining circle events.
    while (!events.empty())
        ProcessEvents();
    PrintOutput();
    FinishEdges();
}

void ClearOutput()
{
    output.clear();
}

void ProcessPoints()
{
    point p = points.top();
    points.pop();

    FrontInsert(p);
}

void FrontInsert(point p)
{
    if (!root)
    {
        root = new arc(p);
        return;
    }

    for (arc *i = root; i; i = i->next)
    {
        point z, zz;
        if (Intersect(p, i, &z))
        {
            if (i->next && !Intersect(p, i->next, &zz))
            {
                i->next->prev = new arc(i->p, i, i->next);
                i->next = i->next->prev;
            }
            else
                i->next = new arc(p, i, i->next);
            i->next->s1 = i->s1;

            // Add p between i and i->next.
            i->next->prev = new arc(p, i, i->next);
            i->next = i->next->prev;

            i = i->next; // Now i points to the new arc.

            // Add new half-edges connected to i's endpoints.
            i->prev->s1 = i->s0 = new seg(z);
            i->next->s0 = i->s1 = new seg(z);

            CheckCircleEvent(i, p.x);
            CheckCircleEvent(i->prev, p.x);
            CheckCircleEvent(i->next, p.x);

            return;
        }
    }
    arc *i;
    for (i = root; i->next; i = i->next)
        ; // Find the last node.

    i->next = new arc(p, i);
    // Insert segment between p and i
    point start;
    start.x = X0;
    start.y = (i->next->p.y + i->p.y) / 2;
    i->s1 = i->next->s0 = new seg(start);
}

void CheckCircleEvent(arc *i, double x0)
{
    // Invalidate any old event.
    if (i->e && i->e->x != x0)
        i->e->valid = false;
    i->e = NULL;

    if (!i->prev || !i->next)
        return;

    double x;
    point o;

    if (Circle(i->prev->p, i->p, i->next->p, &x, &o) && x > x0)
    {
        // Create new event.
        i->e = new event(x, o, i);
        events.push(i->e);
    }
}

// Find the rightmost point on the circle through a,b,c.
bool Circle(point a, point b, point c, double *x, point *o)
{
    // Check that bc is a "right turn" from ab.
    if ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y) > 0)
        return false;

    // Algorithm from O'Rourke 2ed p. 189.
    double A = b.x - a.x, B = b.y - a.y,
           C = c.x - a.x, D = c.y - a.y,
           E = A * (a.x + b.x) + B * (a.y + b.y),
           F = C * (a.x + c.x) + D * (a.y + c.y),
           G = 2 * (A * (c.y - b.y) - B * (c.x - b.x));

    if (G == 0)
        return false; // Points are co-linear.

    // Point o is the center of the circle.
    o->x = (D * E - B * F) / G;
    o->y = (A * F - C * E) / G;

    // o.x plus radius equals max x coordinate.
    *x = o->x + sqrt(pow(a.x - o->x, 2) + pow(a.y - o->y, 2));
    return true;
}

// Will a new parabola at point p intersect with arc i?
bool Intersect(point p, arc *i, point *res)
{
    if (i->p.x == p.x)
        return false;

    double a, b;
    if (i->prev) // Get the intersection of i->prev, i.
        a = Intersection(i->prev->p, i->p, p.x).y;
    if (i->next) // Get the Intersection of i->next, i.
        b = Intersection(i->p, i->next->p, p.x).y;

    if ((!i->prev || a <= p.y) && (!i->next || p.y <= b))
    {
        res->y = p.y;

        // Plug it back into the parabola equation.
        res->x = (i->p.x * i->p.x + (i->p.y - res->y) * (i->p.y - res->y) - p.x * p.x) / (2 * i->p.x - 2 * p.x);

        return true;
    }
    return false;
}

// Where do two parabolas intersect?
point Intersection(point p0, point p1, double l)
{
    point res, p = p0;

    if (p0.x == p1.x)
        res.y = (p0.y + p1.y) / 2;
    else if (p1.x == l)
        res.y = p1.y;
    else if (p0.x == l)
    {
        res.y = p0.y;
        p = p1;
    }
    else
    {
        // Use the quadratic formula.
        double z0 = 2 * (p0.x - l);
        double z1 = 2 * (p1.x - l);

        double a = 1 / z0 - 1 / z1;
        double b = -2 * (p0.y / z0 - p1.y / z1);
        double c = (p0.y * p0.y + p0.x * p0.x - l * l) / z0 - (p1.y * p1.y + p1.x * p1.x - l * l) / z1;

        res.y = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    }
    // Plug back into one of the parabola equations.
    res.x = (p.x * p.x + (p.y - res.y) * (p.y - res.y) - l * l) / (2 * p.x - 2 * l);
    return res;
}

void FinishEdges()
{
    // Advance the sweep line so no parabolas can cross the bounding box.
    double l = X1 + (X1 - X0) + (Y1 - Y0);

    // Extend each remaining segment to the new parabola Intersections.
    for (arc *i = root; i->next; i = i->next)
        if (i->s1)
            i->s1->finish(Intersection(i->p, i->next->p, l * 2));
}

void PrintOutput()
{
    // Bounding box coordinates.

    // Each output segment in four-column format.
    vector<seg *>::iterator i;
    for (i = output.begin(); i != output.end(); i++)
    {
        point p0 = (*i)->start;
        point p1 = (*i)->end;
    }
}

void ProcessEvent()
{
    // Get the next event from the queue.
    event *e = events.top();
    events.pop();

    if (e->valid)
    {
        // Start a new edge.
        seg *s = new seg(e->p);

        // Remove the associated arc from the front.
        arc *a = e->a;
        if (a->prev)
        {
            a->prev->next = a->next;
            a->prev->s1 = s;
        }
        if (a->next)
        {
            a->next->prev = a->prev;
            a->next->s0 = s;
        }

        // Finish the edges before and after a.
        if (a->s0)
            a->s0->finish(e->p);
        if (a->s1)
            a->s1->finish(e->p);

        // Recheck circle events on either side of p:
        if (a->prev)
            CheckCircleEvent(a->prev, e->x);
        if (a->next)
            CheckCircleEvent(a->next, e->x);
    }
    delete e;
}

void ProcessEvents()
{
    // Get the next event from the queue.
    event *e = events.top();
    events.pop();

    if (e->valid)
    {
        // Start a new edge.
        seg *s = new seg(e->p);

        // Remove the associated arc from the front.
        arc *a = e->a;
        if (a->prev)
        {
            a->prev->next = a->next;
            a->prev->s1 = s;
        }
        if (a->next)
        {
            a->next->prev = a->prev;
            a->next->s0 = s;
        }

        // Finish the edges before and after a.
        if (a->s0)
            a->s0->finish(e->p);
        if (a->s1)
            a->s1->finish(e->p);

        // Recheck circle events on either side of p:
        if (a->prev)
            CheckCircleEvent(a->prev, e->x);
        if (a->next)
            CheckCircleEvent(a->next, e->x);
    }
    delete e;
}