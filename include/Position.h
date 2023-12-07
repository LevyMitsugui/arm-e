
class Position{
	private:
	float p[3];

	public:
	Position(float x, float y, float z){
		this->p[0] = x;
		this->p[1] = y;
		this->p[2] = z;
	}
	void setxyz(float x, float y, float z){
		this->p[0] = x;
		this->p[1] = y;
		this->p[2] = z;
	}
	void setx(float x){
		this->p[0] = x;
	}
	void sety(float y){
		this->p[1] = y;
	}
	void setz(float z){
		this->p[2] = z;
	}
	float getx(){
		return this->p[0];
	}
	float gety(){
		return this->p[1];
	}
	float getz(){
		return this->p[2];
	}
	float* getp(){
		return this->p;
	}
};