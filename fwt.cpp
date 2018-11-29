const int MX = 1000;
int fwt[MX];
void add(int i, int d) {
	while (i < MX) {
		fwt[i] += d;
		i += i&(-i);
	}
}
int get(int i) {
	int s = 0;
	while (i) {
		s += fwt[i];
		i -= i&(-i);
	}
	return s;
}
int get_kth(int k) {
	int cur = 0;
	for (int i = 18; i >= 0; i--) {
		if (cur + (1 << i) < MX && fwt[cur + (1 << i)] < k) {
			cur += (1 << i);
			k -= fwt[cur];
		}
	}
	return cur + 1;
}
