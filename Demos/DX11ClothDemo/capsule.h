#include "cap.h"
#include "cylinder.h"


class capsule
{
	public:

	cap topCap;
	cap bottomCap;
	cylinder coreCylinder;

	double x_offset, y_offset, z_offset;
	
	int width;
	int height;
	bool top;

	btCollisionShape *collisionShape;
	btCollisionObject *collisionObject;

	void set_collision_object(btCollisionObject* co)
	{
		collisionObject = co;
		topCap.set_collision_object( co );
		bottomCap.set_collision_object( co );
		coreCylinder.set_collision_object( co );
	}

	void set_collision_shape(btCollisionShape* cs)
	{
		collisionShape = cs;
		topCap.set_collision_shape( cs );
		bottomCap.set_collision_shape( cs );
		coreCylinder.set_collision_shape( cs );
	}

	void create_texture(void)
	{
		topCap.create_texture();
		bottomCap.create_texture();
		coreCylinder.create_texture();
	}

	void destroy()
	{
		topCap.destroy();
		bottomCap.destroy();
		coreCylinder.destroy();
	}

	void draw(void)
	{
		topCap.draw();
		bottomCap.draw();
		coreCylinder.draw();
	}


	// paddingFactor is the amount of padding to allow the capsule collider around the 
	void create_buffers(int width_, int height_)
	{
		topCap.create_buffers(width_, height_, true);
		bottomCap.create_buffers(width_, height_, false);
		coreCylinder.create_buffers(width_, height_);
	}
};
