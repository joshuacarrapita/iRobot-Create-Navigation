#ifndef MOVING_H
#define MOVING_H

class Moving
{
	public:
		void stop();
		void move(double lspeed, double rspeed);
		void turn(double angle);
		void turnRA(int direction);
		void turnAround();
		void moveDist(double dist);
};

#endif
