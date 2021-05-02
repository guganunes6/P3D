/**
 * common.glsl
 * Common types and functions used for ray tracing.
 */

const float pi = 3.14159265358979;
const float epsilon = 0.001;

struct Ray {
    vec3 o;     // origin
    vec3 d;     // direction - always set with normalized vector
    float t;    // time, for motion blur
};

Ray createRay(vec3 o, vec3 d, float t)
{
    Ray r;
    r.o = o;
    r.d = d;
    r.t = t;
    return r;
}

Ray createRay(vec3 o, vec3 d)
{
    return createRay(o, d, 0.0);
}

vec3 pointOnRay(Ray r, float t)
{
    return r.o + r.d * t;
}

float gSeed = 0.0;

uint baseHash(uvec2 p)
{
    p = 1103515245U * ((p >> 1U) ^ (p.yx));
    uint h32 = 1103515245U * ((p.x) ^ (p.y>>3U));
    return h32 ^ (h32 >> 16);
}

float hash1(inout float seed) {
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1,seed += 0.1)));
    return float(n) / float(0xffffffffU);
}

vec2 hash2(inout float seed) {
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1,seed += 0.1)));
    uvec2 rz = uvec2(n, n * 48271U);
    return vec2(rz.xy & uvec2(0x7fffffffU)) / float(0x7fffffff);
}

vec3 hash3(inout float seed)
{
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1, seed += 0.1)));
    uvec3 rz = uvec3(n, n * 16807U, n * 48271U);
    return vec3(rz & uvec3(0x7fffffffU)) / float(0x7fffffff);
}

float rand(vec2 v)
{
    return fract(sin(dot(v.xy, vec2(12.9898, 78.233))) * 43758.5453);
}

vec3 toLinear(vec3 c)
{
    return pow(c, vec3(2.2));
}

vec3 toGamma(vec3 c)
{
    return pow(c, vec3(1.0 / 2.2));
}

vec2 randomInUnitDisk(inout float seed) {
    vec2 h = hash2(seed) * vec2(1.0, 6.28318530718);
    float phi = h.y;
    float r = sqrt(h.x);
	return r * vec2(sin(phi), cos(phi));
}

vec3 randomInUnitSphere(inout float seed)
{
    vec3 h = hash3(seed) * vec3(2.0, 6.28318530718, 1.0) - vec3(1.0, 0.0, 0.0);
    float phi = h.y;
    float r = pow(h.z, 1.0/3.0);
	return r * vec3(sqrt(1.0 - h.x * h.x) * vec2(sin(phi), cos(phi)), h.x);
}

struct Camera
{
    vec3 eye;
    vec3 u, v, n;
    float width, height;
    float lensRadius;
    float planeDist, focusDist;
    float time0, time1;
};

Camera createCamera(
    vec3 eye,
    vec3 at,
    vec3 worldUp,
    float fovy,
    float aspect,
    float aperture,  //diametro em multiplos do pixel size
    float focusDist,  //focal ratio
    float time0,
    float time1)
{
    Camera cam;
    if(aperture == 0.0) cam.focusDist = 1.0; //pinhole camera then focus in on vis plane
    else cam.focusDist = focusDist;
    vec3 w = eye - at;
    cam.planeDist = length(w);
    cam.height = 2.0 * cam.planeDist * tan(fovy * pi / 180.0 * 0.5);
    cam.width = aspect * cam.height;

    cam.lensRadius = aperture * 0.5 * cam.width / iResolution.x;  //aperture ratio * pixel size; (1 pixel=lente raio 0.5)
    cam.eye = eye;
    cam.n = normalize(w);
    cam.u = normalize(cross(worldUp, cam.n));
    cam.v = cross(cam.n, cam.u);
    cam.time0 = time0;
    cam.time1 = time1;
    return cam;
}

Ray getRay(Camera cam, vec2 pixel_sample)  //rnd pixel_sample viewport coordinates
{
    vec2 ls = cam.lensRadius * randomInUnitDisk(gSeed);  //ls - lens sample for DOF
    float time = cam.time0 + hash1(gSeed) * (cam.time1 - cam.time0);
    
    //Calculate eye_offset and ray direction

    vec3 ray_direction;
    vec3 eye_offset;

	vec3 ps = vec3(cam.width * (pixel_sample.x / iResolution.x - 0.5), cam.height * (pixel_sample.y / iResolution.y - 0.5), 0);
    vec3 p = vec3(ps.x * cam.focusDist, ps.y * cam.focusDist, -cam.planeDist * cam.focusDist);

	ray_direction = (cam.u * (p.x - ls.x) + cam.v * (p.y - ls.y) + cam.n * -cam.planeDist * cam.focusDist);

	eye_offset = cam.eye + cam.u * ls.x + cam.v * ls.y;

    return createRay(eye_offset, normalize(ray_direction), time);
}

// MT_ material type
#define MT_DIFFUSE 0
#define MT_METAL 1
#define MT_DIALECTRIC 2

struct Material
{
    int type;
    vec3 albedo;
    float roughness; // controls roughness for metals
    float refIdx; // index of refraction for dialectric
    float refractionRoughness;
};

Material createDiffuseMaterial(vec3 albedo)
{
    Material m;
    m.type = MT_DIFFUSE;
    m.albedo = albedo;
    return m;
}

Material createMetalMaterial(vec3 albedo, float roughness)
{
    Material m;
    m.type = MT_METAL;
    m.albedo = albedo;
    m.roughness = roughness;
    return m;
}

Material createDialectricMaterial(vec3 albedo, float refIdx, float refractionRoughness)
{
    Material m;
    m.type = MT_DIALECTRIC;
    m.albedo = albedo;
    m.refIdx = refIdx;
    m.refractionRoughness = refractionRoughness;
    return m;
}

struct HitRecord
{
    vec3 pos;
    vec3 normal;
    float t;            // ray parameter
    Material material;
};


float schlick(float cosine, float ior_1, float ior_2)
{
    //INSERT YOUR CODE HERE
    float fresnelReflectance = pow((ior_1 - ior_2) / (ior_1 + ior_2), 2.0);
	float Kr = fresnelReflectance + (1.0 - fresnelReflectance) * pow(1.0 - cosine, 5.0);

    return Kr;
}

bool scatter(Ray rIn, HitRecord rec, out vec3 atten, out Ray rScattered)
{
    if(rec.material.type == MT_DIFFUSE)
    {
        //INSERT CODE HERE,
        vec3 hitPoint = rec.pos + rec.normal * 0.001;
        vec3 S = hitPoint + rec.normal + normalize(randomInUnitSphere(gSeed));
        vec3 rayDir = normalize(S - hitPoint);
        rScattered = createRay(hitPoint, rayDir, rIn.t);
        
        atten = rec.material.albedo * max(dot(rScattered.d, rec.normal), 0.0) / pi;
        return true;
    }
    if(rec.material.type == MT_METAL)
    {
       //INSERT CODE HERE, consider fuzzy reflections
		vec3 R = reflect(rIn.d, rec.normal);
        vec3 hitPoint = rec.pos + rec.normal * 0.001;

        vec3 fuzzy_reflected_ray = normalize(R + randomInUnitSphere(gSeed) * rec.material.roughness);
        if(dot(fuzzy_reflected_ray, rec.normal) > 0.0){
            rScattered = createRay(hitPoint, fuzzy_reflected_ray, rIn.t);
        }
        else rScattered = createRay(hitPoint, R, rIn.t);

        atten = rec.material.albedo;
        return true;
    }
    if(rec.material.type == MT_DIALECTRIC)
    {
        float ior_1;
        float ior_2;

        atten = rec.material.albedo;
        vec3 outwardNormal;
        float niOverNt;
        float cosine;

        if(dot(rIn.d, rec.normal) > 0.0) //hit inside
        {
            outwardNormal = -rec.normal;
            niOverNt = rec.material.refIdx;
            ior_1 = rec.material.refIdx;
            ior_2 = 1.0;
            cosine = rec.material.refIdx * dot(rIn.d, rec.normal); 
        }
        else  //hit from outside
        {
            outwardNormal = rec.normal;
            niOverNt = 1.0 / rec.material.refIdx;
            ior_1 = 1.0;
            ior_2 = rec.material.refIdx;
            cosine = -dot(rIn.d, rec.normal); 
        }

        //Use probabilistic math to decide if scatter a reflected ray or a refracted ray

        float reflectProb;

        vec3 V = normalize(rIn.o - rec.pos);
        vec3 VT = outwardNormal * dot(V, outwardNormal) - V;
		float senoT = niOverNt * length(VT);

        //float senI = length(VT);
        //float cosI = sqrt(1.0 - senI * senI);

        if (senoT > 1.0)
		{
			reflectProb = 1.0;
		}
        else
        {
            reflectProb = schlick(cosine, ior_1, ior_2);  
        }
        //reflectProb = 1.0;

        if( hash1(gSeed) < reflectProb) 
        { //Reflection
            vec3 R = reflect(rIn.d, outwardNormal);
            vec3 hitPoint = rec.pos + outwardNormal * 0.001;

            vec3 fuzzy_reflected_ray = normalize(R + randomInUnitSphere(gSeed) * rec.material.roughness);
            if(dot(fuzzy_reflected_ray, outwardNormal) > 0.0){
                rScattered = createRay(hitPoint, fuzzy_reflected_ray, rIn.t);
            }
            else rScattered = createRay(hitPoint, R, rIn.t);
            //atten *= vec3(reflectProb); not necessary since we are only scattering reflectProb rays and not all reflected rays
        }
        else
        {  //Refraction
            vec3 hitPoint = rec.pos - outwardNormal * 0.001;
            

            //refraction calculation
            vec3 refractedRayDirection = refract(rIn.d, outwardNormal, niOverNt);

            vec3 fuzzy_refracted_ray = normalize(refractedRayDirection + randomInUnitSphere(gSeed) * rec.material.refractionRoughness);
            if(dot(fuzzy_refracted_ray, outwardNormal) < 0.0){
                rScattered = createRay(hitPoint, fuzzy_refracted_ray, rIn.t);
            }
            else rScattered = createRay(hitPoint, refractedRayDirection, rIn.t);
            //atten *= vec3(1.0 - reflectProb); not necessary since we are only scattering 1-reflectProb rays and not all refracted rays
        }

        return true;
    }
    return false;
}

struct pointLight {
    vec3 pos;
    vec3 color;
};

pointLight createPointLight(vec3 pos, vec3 color) 
{
    pointLight l;
    l.pos = pos;
    l.color = color;
    return l;
}

struct Triangle {vec3 a; vec3 b; vec3 c; };

Triangle createTriangle(vec3 v0, vec3 v1, vec3 v2)
{
    Triangle t;
    t.a = v0; t.b = v1; t.c = v2;
    return t;
}

bool hit_triangle(Triangle triangle, Ray r, float tmin, float tmax, out HitRecord rec)
{
    //INSERT YOUR CODE HERE
    //calculate a valid t and normal

    //normal
    vec3 edge0 = triangle.b - triangle.a;
	vec3 edge1 = triangle.c - triangle.a;
	vec3 normal = normalize(cross(edge0, edge1));

    //Barycentric coordinates
    float t;
    
	float a = triangle.b.x - triangle.a.x;
	float b = triangle.c.x - triangle.a.x;
	float c = -r.d.x;

	float d = r.o.x - triangle.a.x;

	float e = triangle.b.y - triangle.a.y;
	float f = triangle.c.y - triangle.a.y;
	float g = -r.d.y;

	float h = r.o.y - triangle.a.y;

	float i = triangle.b.z - triangle.a.z;
	float j = triangle.c.z - triangle.a.z;
	float k = -r.d.z;

	float l = r.o.z - triangle.a.z;

	float B = (d * (f * k - g * j) + b * (g * l - h * k) + c * (h * j - f * l)) / 
			  (a * (f * k - g * j) + b * (g * i - e * k) + c * (e * j - f * i));

	float Y = (a * (h * k - g * l) + d * (g * i - e * k) + c * (e * l - h * i)) /
		      (a * (f * k - g * j) + b * (g * i - e * k) + c * (e * j - f * i));

	if (0.0 <= B && B <= 1.0 && 0.0 <= Y && Y <= 1.0 && 0.0 <= B + Y && B + Y <= 1.0)
	{
		t = (a * (f * l - h * j) + b * (h * i - e * l) + d * (e * j - f * i)) /
			(a * (f * k - g * j) + b * (g * i - e * k) + c * (e * j - f * i));

		if(t < tmax && t > tmin)
        {
            rec.t = t;
            rec.normal = normal;
            rec.pos = pointOnRay(r, rec.t);
            return true;
        }
        return false;
    }
	else return false;
}


struct Sphere
{
    vec3 center;
    float radius;
};

Sphere createSphere(vec3 center, float radius)
{
    Sphere s;
    s.center = center;
    s.radius = radius;
    return s;
}


struct MovingSphere
{
    vec3 center0, center1;
    float radius;
    float time0, time1;
};

MovingSphere createMovingSphere(vec3 center0, vec3 center1, float radius, float time0, float time1)
{
    MovingSphere s;
    s.center0 = center0;
    s.center1 = center1;
    s.radius = radius;
    s.time0 = time0;
    s.time1 = time1;
    return s;
}

vec3 center(MovingSphere mvsphere, float time)
{
    return mvsphere.center0 + ((time - mvsphere.time0) / (mvsphere.time1 - mvsphere.time0)) * (mvsphere.center1 - mvsphere.center0);
}


/*
 * The function naming convention changes with these functions to show that they implement a sort of interface for
 * the book's notion of "hittable". E.g. hit_<type>.
 */

bool hit_sphere(Sphere s, Ray r, float tmin, float tmax, out HitRecord rec)
{
    //INSERT YOUR CODE HERE
    //calculate a valid t and normal
    float t;
    float a = (r.d.x * r.d.x) + (r.d.y * r.d.y) + (r.d.z * r.d.z);
	float b = dot((s.center - r.o), r.d);
	float c = dot((s.center - r.o), (s.center - r.o)) - pow(s.radius, 2.0);
	
	//ray origin outside
	if (c > 0.0)
	{
		//sphere is behind the ray
		if (b <= 0.0)
		{
			return false;
		}

		//complex root so no intersection
		if (b * b - c <= 0.0)
		{
			return false;
		}

		//it is outside so we want the smallest root
		t = b - sqrt(b * b - c);
		//return true;
	}
	else
	{
		//it inside so we want the largest root
		t = b + sqrt(b * b - c);
		//return true;
	}
	
    if(t < tmax && t > tmin) {
        rec.t = t;
        rec.pos = pointOnRay(r, rec.t);
        if (s.radius < 0.0) {
            rec.normal = normalize(s.center - rec.pos);
        }
        else {
            rec.normal = normalize(rec.pos - s.center);
        }
        return true;
    }
    else return false;
}

bool hit_movingSphere(MovingSphere s, Ray r, float tmin, float tmax, out HitRecord rec)
{

     //INSERT YOUR CODE HERE
     //Calculate the moving center
    //calculate a valid t and normal
	float t;
    float a = (r.d.x * r.d.x) + (r.d.y * r.d.y) + (r.d.z * r.d.z);
	float b = dot((center(s, r.t) - r.o), r.d);
	float c = dot((center(s, r.t) - r.o), (center(s, r.t) - r.o)) - pow(s.radius, 2.0);
	
	//ray origin outside
	if (c > 0.0)
	{
		//sphere is behind the ray
		if (b <= 0.0)
		{
			return false;
		}

		//complex root so no intersection
		if (b * b - c <= 0.0)
		{
			return false;
		}

		//it is outside so we want the smallest root
		t = b - sqrt(b * b - c);
		//return true;
	}
	else
	{
		//it inside so we want the largest root
		t = b + sqrt(b * b - c);
		//return true;
	}
	
    if(t < tmax && t > tmin) {
        rec.t = t;
        rec.pos = pointOnRay(r, rec.t);
        rec.normal = normalize(rec.pos - center(s, r.t));
        return true;
    }
    else return false;
    
}
