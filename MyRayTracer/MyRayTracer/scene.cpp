#include <iostream>
#include <string>
#include <fstream>
#include <IL/il.h>

#include "maths.h"
#include "scene.h"


Triangle::Triangle(Vector& P0, Vector& P1, Vector& P2)
{
	points[0] = P0; points[1] = P1; points[2] = P2;

	/* Calculate the normal */
	normal = Vector(0, 0, 0);
	Vector edge0 = P1 - P0;
	Vector edge1 = P2 - P0;
	normal = edge0 % edge1;
	normal.normalize();

	//Calculate the Min and Max for bounding box
	Max = Vector(10000, +10000, +10000);
	Min = Vector(-10000, -10000, -10000);

	float xmax = P0.x > P1.x ? (P0.x > P2.x ? P0.x : P2.x) : (P1.x > P2.x ? P1.x : P2.x);
	float ymax = P0.y > P1.y ? (P0.y > P2.y ? P0.y : P2.y) : (P1.y > P2.y ? P1.y : P2.y);
	float zmax = P0.z > P1.z ? (P0.z > P2.z ? P0.z : P2.z) : (P1.z > P2.z ? P1.z : P2.z);
	float xmin = P0.x < P1.x ? (P0.x < P2.x ? P0.x : P2.x) : (P1.x < P2.x ? P1.x : P2.x);
	float ymin = P0.y < P1.y ? (P0.y < P2.y ? P0.y : P2.y) : (P1.y < P2.y ? P1.y : P2.y);
	float zmin = P0.z < P1.z ? (P0.z < P2.z ? P0.z : P2.z) : (P1.z < P2.z ? P1.z : P2.z);

	Min = Vector(xmin, ymin, zmin);
	Max = Vector(xmax, ymax, zmax);

	// enlarge the bounding box a bit just in case...
	Min -= EPSILON;
	Max += EPSILON;
}

AABB Triangle::GetBoundingBox() {
	return(AABB(Min, Max));
}

Vector Triangle::getNormal(Vector point)
{
	return normal;
}

//
// Ray/Triangle intersection test using Tomas Moller-Ben Trumbore algorithm.
//

bool Triangle::intercepts(Ray& r, float& t ) {
	float a = points[1].x - points[0].x;
	float b = points[2].x - points[0].x;
	float c = -r.direction.x;

	float d = r.origin.x - points[0].x;

	float e = points[1].y - points[0].y;
	float f = points[2].y - points[0].y;
	float g = -r.direction.y;

	float h = r.origin.y - points[0].y;

	float i = points[1].z - points[0].z;
	float j = points[2].z - points[0].z;
	float k = -r.direction.z;

	float l = r.origin.z - points[0].z;

	float B = (d * (f * k - g * j) + b * (g * l - h * k) + c * (h * j - f * l)) / 
			  (a * (f * k - g * j) + b * (g * i - e * k) + c * (e * j - f * i));

	float Y = (a * (h * k - g * l) + d * (g * i - e * k) + c * (e * l - h * i)) /
		      (a * (f * k - g * j) + b * (g * i - e * k) + c * (e * j - f * i));

	if (0 <= B && B <= 1 && 0 <= Y && Y <= 1 && 0 <= B + Y && B + Y <= 1)
	{
		t = (a * (f * l - h * j) + b * (h * i - e * l) + d * (e * j - f * i)) /
			(a * (f * k - g * j) + b * (g * i - e * k) + c * (e * j - f * i));

		if (t < 0) return false;
		else return true;
	}

	else return false;
}

Plane::Plane(Vector& a_PN, float a_D)
	: PN(a_PN), D(a_D)
{}

Plane::Plane(Vector& P0, Vector& P1, Vector& P2)
{
   float l;

   //Calculate the normal plane: counter-clockwise vectorial product.
   PN = Vector(0, 0, 0);	
   Vector edge0 = P0 - P1;
   Vector edge1 = P0 - P2;
   PN = edge0 % edge1;

   if ((l=PN.length()) == 0.0)
   {
     cerr << "DEGENERATED PLANE!\n";
   }
   else
   {
     PN.normalize();
	 //Calculate D
	 D = (PN.x * P0.x + PN.y * P0.y + PN.z * P0.z);
   }
}

//
// Ray/Plane intersection test.
//

bool Plane::intercepts( Ray& r, float& t )
{
	if (PN * r.direction == 0)
	{
		return (false);
	}
	else
	{
		t = (D - r.origin * PN) / (r.direction * PN);
		return (t > 0);
	}
}

Vector Plane::getNormal(Vector point) 
{
  return PN;
}


bool Sphere::intercepts(Ray& r, float& t )
{
	float a = (r.direction.x * r.direction.x) + (r.direction.y * r.direction.y) + (r.direction.z * r.direction.z);
	float b = (center - r.origin) * r.direction;
	float c = (center - r.origin) * (center - r.origin) - SqRadius;
	

	if (c > 0)
	{
		if (b <= 0)
		{
			return false;
		}

		if (b * b - c <= 0)
		{
			return false;
		}

		t = b - sqrt(b * b - c);
		return true;
	}
	else
	{
		t = b + sqrt(b * b - c);
		return true;
	}
}


Vector Sphere::getNormal( Vector point )
{
	Vector normal = point - center;
	return (normal.normalize());
}

AABB Sphere::GetBoundingBox() {
	Vector a_min;
	Vector a_max;

	a_min.x = center.x - radius;
	a_min.y = center.y - radius;
	a_min.z = center.z - radius;
	a_max.x = center.x + radius;
	a_max.y = center.y + radius;
	a_max.z = center.z + radius;

	return(AABB(a_min, a_max));
}

aaBox::aaBox(Vector& minPoint, Vector& maxPoint) //Axis aligned Box: another geometric object
{
	this->min = minPoint;
	this->max = maxPoint;
}

AABB aaBox::GetBoundingBox() {
	return(AABB(min, max));
}

bool aaBox::intercepts(Ray& ray, float& t)
{
	Vector o = ray.origin;
	Vector d = ray.direction;

	Vector t_min;
	Vector t_max;

	//calculate intersection
	double a = 1.0f / d.x;
	if (a >= 0)
	{
		t_min.x = (min.x - o.x) * a;
		t_max.x = (max.x - o.x) * a;
	}
	else
	{
		t_min.x = (max.x - o.x) * a;
		t_max.x = (min.x - o.x) * a;
	}

	double b = 1.0f / d.y;
	if (b >= 0)
	{
		t_min.y = (min.y - o.y) * b;
		t_max.y = (max.y - o.y) * b;
	}
	else
	{
		t_min.y = (max.y - o.y) * b;
		t_max.y = (min.y - o.y) * b;
	}

	double c = 1.0f / d.z;
	if (c >= 0)
	{
		t_min.z = (min.z - o.z) * c;
		t_max.z = (max.z - o.z) * c;
	}
	else
	{
		t_min.z = (max.z - o.z) * c;
		t_max.z = (min.z - o.z) * c;
	}

	float tE;
	float tL;
	Vector face_in;
	Vector face_out;

	//find largest tE, entering tValue

	if (t_min.x > t_min.y)
	{
		tE = t_min.x;
		face_in = (a >= 0) ? Vector(-1, 0, 0) : Vector(1, 0, 0);
	}

	else
	{
		tE = t_min.y;
		face_in = (b >= 0) ? Vector(0, -1, 0) : Vector(0, 1, 0);
	}

	if (t_min.z > tE)
	{
		tE = t_min.z;
		face_in = (c >= 0) ? Vector(0, 0, -1) : Vector(0, 0, 1);
	}

	//find smallest tL, leaving tValue

	if (t_max.x < t_max.y)
	{
		tL = t_max.x;
		face_out = (a >= 0) ? Vector(1, 0, 0) : Vector(-1, 0, 0);
	}

	else
	{
		tL = t_max.y;
		face_out = (b >= 0) ? Vector(0, 1, 0) : Vector(0, -1, 0);
	}

	if (t_max.z < tL)
	{
		tL = t_max.z;
		face_out = (c >= 0) ? Vector(0, 0, 1) : Vector(0, 0, -1);
	}

	//condition for a hit

	if (tE < tL && tL > 0)
	{
		if (tE > 0)
		{
			t = tE; // ray hits outside surface
			Normal = face_in;
		}

		else
		{
			t = tL; //ray hits inside surface;
			Normal = face_out;
		}
		return true;
	}
	else return false;
}

Vector aaBox::getNormal(Vector point)
{
	return Normal;
}

Scene::Scene()
{}

Scene::~Scene()
{
	/*for ( int i = 0; i < objects.size(); i++ )
	{
		delete objects[i];
	}
	objects.erase();
	*/
}

int Scene::getNumObjects()
{
	return objects.size();
}


void Scene::addObject(Object* o)
{
	objects.push_back(o);
}


Object* Scene::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}


int Scene::getNumLights()
{
	return lights.size();
}


void Scene::addLight(Light* l)
{
	lights.push_back(l);
}


Light* Scene::getLight(unsigned int index)
{
	if (index >= 0 && index < lights.size())
		return lights[index];
	return NULL;
}

void Scene::LoadSkybox(const char *sky_dir)
{
	char *filenames[6];
	char buffer[100];
	const char *maps[] = { "/right.jpg", "/left.jpg", "/top.jpg", "/bottom.jpg", "/front.jpg", "/back.jpg" };

	for (int i = 0; i < 6; i++) {
		strcpy_s(buffer, sizeof(buffer), sky_dir);
		strcat_s(buffer, sizeof(buffer), maps[i]);
		filenames[i] = (char *)malloc(sizeof(buffer));
		strcpy_s(filenames[i], sizeof(buffer), buffer);
	}
	
	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++) {
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i]))  //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte *)malloc(size);
		ILubyte *bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

Color Scene::GetSkyboxColor(Ray& r) {
	float t_intersec;
	Vector cubemap_coords; //To index the skybox

	float ma;
	CubeMap img_side;
	float sc, tc, s, t;
	unsigned int xp, yp, width, height, bytesperpixel;

	//skybox indexed by the ray direction
	cubemap_coords = r.direction;


	if (fabs(cubemap_coords.x) > fabs(cubemap_coords.y)) {
		ma = fabs(cubemap_coords.x);
		cubemap_coords.x >= 0 ? img_side = LEFT : img_side = RIGHT;    //left cubemap at X = +1 and right at X = -1
	}
	else {
		ma = fabs(cubemap_coords.y);
		cubemap_coords.y >= 0 ? img_side = TOP : img_side = BOTTOM; //top cubemap at Y = +1 and bottom at Y = -1
	}

	if (fabs(cubemap_coords.z) > ma) {
		ma = fabs(cubemap_coords.z);
		cubemap_coords.z >= 0 ? img_side = FRONT : img_side = BACK;   //front cubemap at Z = +1 and back at Z = -1
	}

	switch (img_side) {

	case 0:  //right
		sc = -cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 1:  //left
		sc = cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 2:  //top
		sc = -cubemap_coords.x;
		tc = -cubemap_coords.z;
		break;

	case 3: //bottom
		sc = -cubemap_coords.x;
		tc = cubemap_coords.z;
		break;

	case 4:  //front
		sc = -cubemap_coords.x;
		tc = cubemap_coords.y;
		break;

	case 5: //back
		sc = cubemap_coords.x;
		tc = cubemap_coords.y;
		break;
	}

	double invMa = 1 / ma;
	s = (sc * invMa + 1) / 2;
	t = (tc * invMa + 1) / 2;

	width = skybox_img[img_side].resX;
	height = skybox_img[img_side].resY;
	bytesperpixel = skybox_img[img_side].BPP;

	xp = int((width - 1) * s);
	xp < 0 ? 0 : (xp > (width - 1) ? width - 1 : xp);
	yp = int((height - 1) * t);
	yp < 0 ? 0 : (yp > (height - 1) ? height - 1 : yp);

	float red = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel]);
	float green = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 1]);
	float blue = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 2]);

	return(Color(red, green, blue));
}




////////////////////////////////////////////////////////////////////////////////
// P3F file parsing methods.
//
void next_token(ifstream& file, char *token, const char *name)
{
  file >> token;
  if (strcmp(token, name))
    cerr << "'" << name << "' expected.\n";
}

bool Scene::load_p3f(const char *name)
{
  const	int	lineSize = 1024;
  string	cmd;
  char		token	[256];
  ifstream	file(name, ios::in);
  Material *	material;

  material = NULL;

  if (file >> cmd)
  {
    while (true)
    {
      
	  if (cmd == "f")   //Material
      {
	    double Kd, Ks, Shine, T, ior;
	    Color cd, cs;

	    file >> cd >> Kd >> cs >> Ks >> Shine >> T >> ior;

	    material = new Material(cd, Kd, cs, Ks, Shine, T, ior);
      }

      else if (cmd == "s")    //Sphere
      {
	     Vector center;
    	 float radius;
         Sphere* sphere;

	    file >> center >> radius;
        sphere = new Sphere(center,radius);
	    if (material) sphere->SetMaterial(material);
        this->addObject( (Object*) sphere);
      }

	  else if (cmd == "box")    //axis aligned box
	  {
		  Vector minpoint, maxpoint;
		  aaBox	*box;

		  file >> minpoint >> maxpoint;
		  box = new aaBox(minpoint, maxpoint);
		  if (material) box->SetMaterial(material);
		  this->addObject((Object*)box);
	  }
	  else if (cmd == "p")  // Polygon: just accepts triangles for now
      {
		  Vector P0, P1, P2;
		  Triangle* triangle;
		  unsigned total_vertices;
		  
		  file >> total_vertices;
		  if (total_vertices == 3)
		  {
			  file >> P0 >> P1 >> P2;
			  triangle = new Triangle(P0, P1, P2);
			  if (material) triangle->SetMaterial(material);
			  this->addObject( (Object*) triangle);
		  }
		  else
		  {
			  cerr << "Unsupported number of vertices.\n";
			  break;
		  }
      }
      
	  else if (cmd == "mesh") {
		  unsigned total_vertices, total_faces;
		  unsigned P0, P1, P2;
		  Triangle* triangle;
		  Vector* verticesArray, vertex;

		  file >> total_vertices >> total_faces;
		  verticesArray = (Vector*)malloc(total_vertices * sizeof(Vector));
		  for (int i = 0; i < total_vertices; i++) {
			  file >> vertex;
			  verticesArray[i] = vertex;
		  }
		  for (int i = 0; i < total_faces; i++) {
			  file >> P0 >> P1 >> P2;
			  triangle = new Triangle(verticesArray[P0 - 1], verticesArray[P1 - 1], verticesArray[P2 - 1]); //vertex index start at 1
			  if (material) triangle->SetMaterial(material);
			  this->addObject((Object*)triangle);
		  }

	  }

	  else if (cmd == "pl")  // General Plane
	  {
          Vector P0, P1, P2;
		  Plane* plane;

          file >> P0 >> P1 >> P2;
          plane = new Plane(P0, P1, P2);
	      if (material) plane->SetMaterial(material);
          this->addObject( (Object*) plane);
	  }

      else if (cmd == "l")  // Need to check light color since by default is white
      {
	    Vector pos;
        Color color;

	    file >> pos >> color;
	    
	      this->addLight(new Light(pos, color));
	    
      }
      else if (cmd == "v")
      {
	    Vector up, from, at;
	    float fov, hither;
	    int xres, yres;
        Camera* camera;
		float focal_ratio; //ratio beteween the focal distance and the viewplane distance
		float aperture_ratio; // number of times to be multiplied by the size of a pixel

	    next_token (file, token, "from");
	    file >> from;

	    next_token (file, token, "at");
	    file >> at;

	    next_token (file, token, "up");
	    file >> up;

	    next_token (file, token, "angle");
	    file >> fov;

	    next_token (file, token, "hither");
	    file >> hither;

	    next_token (file, token, "resolution");
	    file >> xres >> yres;

		next_token(file, token, "aperture");
		file >> aperture_ratio;

		next_token(file, token, "focal");
		file >> focal_ratio;
	    // Create Camera
		camera = new Camera( from, at, up, fov, hither, 100.0*hither, xres, yres, aperture_ratio, focal_ratio);
        this->SetCamera(camera);
      }

      else if (cmd == "bclr")   //Background color
      {
		Color bgcolor;
		file >> bgcolor;
		this->SetBackgroundColor(bgcolor);
	  }
	
	  else if (cmd == "env")
	  {
		  file >> token;
		  
		  this->LoadSkybox(token);
		  this->SetSkyBoxFlg(true);
	  }
      else if (cmd[0] == '#')
      {
	    file.ignore (lineSize, '\n');
      }
      else
      {
	    cerr << "unknown command '" << cmd << "'.\n";
	    break;
      }
      if (!(file >> cmd))
        break;
    }
  }

  file.close();
  return true;
};


void Scene::create_random_scene() {
	Camera* camera;
	Material* material;
	Sphere* sphere;

	set_rand_seed(time(NULL) * time(NULL) * time(NULL));
	material = NULL;
	this->SetSkyBoxFlg(false);  //init with no skybox

	this->SetBackgroundColor(Color(0.5, 0.7, 1.0));
	//this->LoadSkybox("skybox");
	//this->SetSkyBoxFlg(true);
	
	camera = new Camera(Vector(13.0, 2.0, 3.0), Vector(0.0, 0.0, 0), Vector(0.0, 1.0, 0.0), 45.0, 0.01, 10000.0, 800, 600, 0, 1.5f);
	this->SetCamera(camera);

	this->addLight(new Light(Vector(7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(-7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(0, 10, 7), Color(1.0, 1.0, 1.0)));

	material = new Material(Color(0.5, 0.5, 0.5), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);


	sphere = new Sphere(Vector(0.0, -1000, 0.0), 1000.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	for (int a = -5; a < 5; a++)
		for (int b = -5; b < 5; b++) {

			double choose_mat = rand_double();

			Vector center = Vector(a + 0.9 * rand_double(), 0.2, b + 0.9 * rand_double());

			if ((center - Vector(4.0, 0.2, 0.0)).length() > 0.9) {
				if (choose_mat < 0.4) {  //diffuse
					material = new Material(Color(rand_double(), rand_double(), rand_double()), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else if (choose_mat < 0.9) {   //metal
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(rand_double(0.5, 1), rand_double(0.5, 1), rand_double(0.5, 1)), 1.0, 220, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else {   //glass 
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}

			}

		}

	material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
	sphere = new Sphere(Vector(0.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.9, Color(1.0, 1.0, 1.0), 0.1, 10, 0, 1.0);
	sphere = new Sphere(Vector(-4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.0, Color(0.7, 0.6, 0.5), 1.0, 220, 0, 1.0);
	sphere = new Sphere(Vector(4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);
}
