
template<class T>
bool check_range(T value, T min, T max) {
    return (value >= min) && (value <= max);
}

#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <dirent.h>



#define RED_H_MIN 0.0
#define RED_H_MAX 20.0
#define RED_H_MIN2 340.0
#define RED_H_MAX2 360.0
//#define RED_S_MIN 125.0
//#define RED_S_MAX 255.0
//#define RED_V_MIN 90.0
//#define RED_V_MAX 255.0

#define ORANGE_H_MIN 20.0
#define ORANGE_H_MAX 30.0
//#define ORANGE_S_MIN 110.0
//#define ORANGE_S_MAX 255.0
//#define ORANGE_V_MIN 140.0
//#define ORANGE_V_MAX 255.0

#define YELLOW_H_MIN 30.0
#define YELLOW_H_MAX 70.0
//#define YELLOW_S_MIN 110.0
//#define YELLOW_S_MAX 255.0
//#define YELLOW_V_MIN 50.0
//#define YELLOW_V_MAX 255.0

#define GREEN_H_MIN 70.0
#define GREEN_H_MAX 160.0
//#define GREEN_S_MIN 55.0
//#define GREEN_S_MAX 255.0
//#define GREEN_V_MIN 80.0
//#define GREEN_V_MAX 255.0

#define BLUE_H_MIN 160.0
#define BLUE_H_MAX 250.0
//#define BLUE_S_MIN 55.0
//#define BLUE_S_MAX 255.0
//#define BLUE_V_MIN 87.0
//#define BLUE_V_MAX 255.0

#define PURPLE_H_MIN 250.0
#define PURPLE_H_MAX 300.0
//#define PURPLE_S_MIN 40.0
//#define PURPLE_S_MAX 255.0
//#define PURPLE_V_MIN 125.0
//#define PURPLE_V_MAX 255.0

#define PINK_H_MIN 300.0
#define PINK_H_MAX 340.0
//#define PINK_S_MIN 40.0
//#define PINK_S_MAX 255.0
//#define PINK_V_MIN 125.0
//#define PINK_V_MAX 255.0

//#define WHITE_H_MIN 0.0
//#define WHITE_H_MAX 255.0
#define WHITE_S_MIN 0.0
#define WHITE_S_MAX 35.0
//#define WHITE_V_MIN 130.0
//#define WHITE_V_MAX 255.0

#define BLACK_V_MIN 0.0
#define BLACK_V_MAX 30.0



using namespace std;

#include <pcl/point_types.h>

int numPCDFiles( const char* path )
{
	int num = 0;
   DIR* dirFile = opendir( path );
   if ( dirFile ) 
   {
      struct dirent* hFile;
      errno = 0;
      while (( hFile = readdir( dirFile )) != NULL ) 
      {
         if ( !strcmp( hFile->d_name, "."  )) continue;
         if ( !strcmp( hFile->d_name, ".." )) continue;

         // in linux hidden files all start with '.'
         //if ( gIgnoreHidden && ( hFile->d_name[0] == '.' )) continue;

         // dirFile.name is the name of the file. Do whatever string comparison 
         // you want here. Something like:
         if ( strstr( hFile->d_name, ".pcd" ))
            num++;
      } 
      closedir( dirFile );
   }
   return num;
}

const char *vinit[] = {"white", "black", "red", "orange", "yellow", "green", "blue", "purple", "pink"};


typedef struct {
    double r;       // percent
    double g;       // percent
    double b;       // percent
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // percent
    double v;       // percent
} hsv;

hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = (max/255.0)*100;                                // v
    delta = max - min;
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max)*100.0;                  // s
    } else {
        // if max is 0, then r = g = b = 0              
            // s = 0, v is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}


rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}



int main()
{
	
	ofstream colorFile;
	colorFile.open("/home/kyle/catkin_ws/src/baxter_controller/scripts/OPE-Release/ObjectColors.txt", std::ofstream::out | std::ofstream::trunc);
	colorFile.close();
	colorFile.open("/home/kyle/catkin_ws/src/baxter_controller/scripts/OPE-Release/ObjectColors.txt",ios::out | ios::app);
	
	int numpcd = numPCDFiles("/home/kyle/catkin_ws/src/baxter_controller/scripts/OPE-Release/");
    vector <string> colors(vinit, vinit+9);
    vector <vector <int> > objs(numpcd, vector<int> (9, 0) );

	for(int i = 0; i < numpcd; i++)
	{
		
		ifstream pcdFile;
		stringstream ss;
		ss << "/home/kyle/catkin_ws/src/baxter_controller/scripts/OPE-Release/object_" << i << ".pcd";
		string filename = ss.str();
		pcdFile.open(filename.c_str(), std::fstream::in);
		
		
		int errors = 0;
		double r_sum, g_sum, b_sum;

		if(pcdFile.is_open())
		{
			cout << "Opening PCD file " << i << endl;
			string line;
			int numlines = 0;
			int filesize = 0;
			for(numlines = 0; numlines < 11; numlines++)
			{
		    	std::getline(pcdFile, line);
		    	istringstream in(line);
		    	string val;
		    	in >> val;
		    	//cout << "val: " << val << endl;
		    	if(val == "POINTS")
		    		in >> filesize;
		    	//cout << filesize << endl;
			
			}
			if(filesize != 0)
			{
				for(int j = 0; j < filesize; j++)
				{
			        std::getline(pcdFile, line);
			        istringstream in(line);
			    
			        float x = 0.0;
			        float y = 0.0;
			        float z = 0.0;
			        float color_float = 0.0;

			        in >> x >> y >> z >> color_float;
			        //cout << x << " " << y << " " << z << " " << color_float << endl;

			        uint32_t val = *reinterpret_cast<int*>(&color_float);
			        uint8_t r = (val >> 16) & 0x0000ff;
					uint8_t g = (val >> 8) & 0x0000ff;
					uint8_t b = (val) & 0x0000ff;

					int r_i = int(r);
					int g_i = int(g);
					int b_i = int(b);
			        //r_sum += r_i;
			        //b_sum += b_i;
			        //g_sum += g_i;

                    rgb rgbPix;
                    rgbPix.r = r_i;
                    rgbPix.g = g_i;
                    rgbPix.b = b_i;
                    hsv hsvPix = rgb2hsv(rgbPix);

                    if(check_range(hsvPix.s, WHITE_S_MIN, WHITE_S_MAX))
                            objs[i][0] += 1;
                    else if(check_range(hsvPix.v, BLACK_V_MIN, BLACK_V_MAX))
                                objs[i][1] += 1;
                    else if(check_range(hsvPix.h, RED_H_MIN, RED_H_MAX))
                                objs[i][2] += 1;
                    else if(check_range(hsvPix.h, RED_H_MIN2, RED_H_MAX2))
                                objs[i][2] += 1;
                    else if(check_range(hsvPix.h, ORANGE_H_MIN, ORANGE_H_MAX))
                                objs[i][3] += 1;
                    else if(check_range(hsvPix.h, YELLOW_H_MIN, YELLOW_H_MAX))
                                objs[i][4] += 1;
                    else if(check_range(hsvPix.h, GREEN_H_MIN, GREEN_H_MAX))
                                objs[i][5] += 1;
                    else if(check_range(hsvPix.h, BLUE_H_MIN, BLUE_H_MAX))
                                objs[i][6] += 1;
                    else if(check_range(hsvPix.h, PURPLE_H_MIN, PURPLE_H_MAX))
                                objs[i][7] += 1;
                    else if(check_range(hsvPix.h, PINK_H_MIN, PINK_H_MAX))
                                objs[i][8] += 1;
				   
				}

                int max = 0;
                string color_id;
                for(int n = 0; n < 9; n++)
                {
                    if(objs[i][n] > max)
                    {
                        max = objs[i][n];
                        color_id = colors[n];
                    }
                }

                //colorFile << "Object: " << i << endl;
                int max_val = 0;
                string max_color;
                for(int c = 0; c < 9; c++)
                {   
                    cout << colors[c] << " " << objs[i][c] << endl;
                    //colorFile << colors[c] << " " << objs[i][c] << endl;
                }

                colorFile << color_id << endl;
                cout << "Detected Color: " << color_id << endl;
                cout << endl;

               
                /*
				double rval = r_sum/float(filesize);
			    double gval = g_sum/float(filesize);
			    double bval = b_sum/float(filesize);

			    rgb rgbColor;
			    rgbColor.r = rval;
			    rgbColor.g = gval;
			    rgbColor.b = bval;

			    hsv hsvColor = rgb2hsv(rgbColor);

			    //cout << rgbColor.r << " " << rgbColor.g << " " << rgbColor.b << endl;
			    //cout << hsvColor.h << " " << hsvColor.s << " " << hsvColor.v << endl;
                
			    string color_id = "none";
                if(check_range(hsvColor.s, WHITE_S_MIN, WHITE_S_MAX))
                            color_id = "white";
                else if(check_range(hsvColor.v, BLACK_V_MIN, BLACK_V_MAX))
                            color_id = "black";
			    else if(check_range(hsvColor.h, RED_H_MIN, RED_H_MAX))
			    			color_id = "red";
			   	else if(check_range(hsvColor.h, ORANGE_H_MIN, ORANGE_H_MAX))
			    			color_id = "orange";
			    else if(check_range(hsvColor.h, YELLOW_H_MIN, YELLOW_H_MAX))
			    			color_id = "yellow";
			    else if(check_range(hsvColor.h, GREEN_H_MIN, GREEN_H_MAX))
			    			color_id = "green";
			    else if(check_range(hsvColor.h, BLUE_H_MIN, BLUE_H_MAX))
			    			color_id = "blue";
			    else if(check_range(hsvColor.h, PURPLE_H_MIN, PURPLE_H_MAX))
			    			color_id = "purple";
                else if(check_range(hsvColor.h, PINK_H_MIN, PINK_H_MAX))
                            color_id = "pink";
            
			    cout << "Object Color: " << color_id << endl;
		        colorFile << "Object " << i << " HSV: " << hsvColor.h << " " << hsvColor.s << " " << hsvColor.v << " Color: " << color_id << endl;
		          */

            }

		}
	}




 

    return 0;
}