#ifndef quart_h_
#define quart_h_

struct Quart//erion
{
	float & q1() { return q[0]; }
	float & q2() { return q[1]; }
	float & q3() { return q[2]; }
	float & q4() { return q[3]; }

protected:
	float q[4];
};

struct ErrorIntegral : protected Quart
{
	float & e1() { return q1(); }
	float & e2() { return q2(); }
	float & e3() { return q3(); }
	float & e4() { return q4(); }
};

#endif
