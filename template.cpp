#include <bits/stdc++.h>
using namespace std;
typedef long long ll;

const double EPS = 1e-8;

struct Point {
	double x, y;
};

bool operator< (Point a, Point b) {
	if (a.x == b.x) {
		return a.y < b.y;
	}
	return a.x < b.x;
}

Point operator+(Point a, Point b) {
	a.x += b.x;
	a.y += b.y;
	return a;
}
Point operator-(Point a, Point b) {
	a.x -= b.x;
	a.y -= b.y;
	return a;
}
Point operator*(Point a, double t) {
	a.x *= t;
	a.y *= t;
	return a;
}

ll cp(Point u, Point v) { // positive === counter-colckwise
	return u.x * v.y - u.y * v.x;
}

vector<Point> convexHull(vector<Point> a)
{
	if (a.size() == 1) return a;
	
	sort(a.begin(), a.end());
	
	vector<Point> up, down;
	up.push_back(a[0]);
	up.push_back(a[1]);
	down.push_back(a[0]);
	down.push_back(a[1]);

	for (int i = 2; i < (int)a.size(); i++) {
		while (up.size() > 1 && cp(up.back() - up[up.size()-2], a[i] - up.back()) > 0) {
			up.pop_back();
		}
		up.push_back(a[i]);
		while (down.size() > 1 && cp(down.back() - down[down.size()-2], a[i] - down.back()) < 0) {
			down.pop_back();
		}
		down.push_back(a[i]);
	}
	
	for (int i = down.size() - 2; i > 0; i--) {
		up.push_back(down[i]);
	}
	return up;
}

Point ortho(Point a) { // clockwise
	return {a.y, -a.x};
}

struct Line {
	Point a, v;
};

Point intersect(Line A, Line B) // A.a + A.v * t == B.a + B.v * T
{
	double t = (cp(B.a, B.v) - cp(A.a, B.v)) / cp(A.v, B.v);
	double T = (cp(A.a, A.v) - cp(B.a, A.v)) / cp(B.v, A.v);
	
	if (-EPS < T && T < 1 + EPS && -EPS < t && t < 1 + EPS) {
		// Segments cross
	}
	
	return A.a + A.v * t;
}

Point CircleCenter(Point& A, Point& B, Point& C)
{
	Line X, Y;
	X.a = (A + B) * 0.5;
	Y.a = (A + C) * 0.5;
	X.v = ortho(B - A);
	Y.v = ortho(C - A);
	return intersect(X, Y);
}

vector<Point> LineCircleIntersection(double x1, double y1, double x2, double y2, double r) // Center: (0, 0)
{ 	
	// x = D * dy +- sgn(dy) * dx * sqrt(r*r*dr*dr-D*D)    /   dr*dr
	// y = -D * dx +- abs(dy) * sqrt(r*r*dr*dr-D*D)   /  dr*dr
	double dx = x2 - x1;
	double dy = y2 - y1;
	double dr = dx*dx + dy*dy;
	double D = x1*y2 - x2*y1;
	
	double DET = r*r * dr - D * D;
	
	vector<Point> res;
	if (DET < -EPS) return res;
	if (DET < EPS) { // Tangent
		res.push_back({D * dy / dr, -D * dx / dr});
	}
	else { // Secant
		DET = sqrt(DET);
		
		double a = (dy < 0 ? -1 : 1) * dx * DET;
		double b = abs(dy) * DET;
		
		res.push_back({(D * dy + a) / dr, (-D * dx + b) / dr});
		res.push_back({(D * dy - a) / dr, (-D * dx - b) / dr});
	}
	return res;
}

vector<Point> CircleIntersection(Point A, Point B, long double R1, long double R2)
{
	vector<Point> ans;
	
	long double d = len(A - B);
	long double x = (d*d + R1*R1 - R2*R2) / (2 * d);
	
	Point V = B - A;
	V.x /= d;
	V.y /= d;
	
	double tmp = R1*R1 - x*x;
	if (tmp < -EPS) return ans;
	if (tmp < EPS) tmp = 0;
	
	long double y = sqrt(tmp);
	
	Point m = A + x*V;
	
	ans.push_back(m + y*ortho(V));
	ans.push_back(m - y*ortho(V));
	return ans;
}

// Closest point pair
struct Point {
	ll x, y;
};
bool operator< (Point a, Point b) {
	return a.x < b.x;
}

ll dist(Point& a, Point& b) {
	return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y);
}

Point strip[maxn];
ll stripClosest(int n, ll d)
{
	for (int i = 0; i < n; i++) {
		for (int j = i + 1; j < n && (strip[j].y - strip[i].y)*(strip[j].y - strip[i].y) < d; j++) {
			d = min(d, dist(strip[i], strip[j]));
		}
	}
	return d;
}

ll closest(Point P[], int n) {
	if (n == 1) return inf;
	
	int m = n / 2;
	Point mp = P[m];
	ll d = min(closest(P, m), closest(P + m, n - m));
	
	for (int t = 0, i = 0, j = m; t < n; t++) {
		if (i < m && (j == n || P[i].y < P[j].y)) {
			strip[t] = P[i++];
		}
		else {
			strip[t] = P[j++];
		}
	}
	for (int t = 0; t < n; t++) {
		P[t] = strip[t];
	}
	
	int t = 0;
	for (int i = 0; i < n; i++) {
		if ((P[i].x - mp.x)*(P[i].x - mp.x) < d) {
			strip[t++] = P[i];
		}
	}
	
	return min(d, stripClosest(t, d));
}

