/** @doc This object implements a visual process to get a disparity map.

    Copyright (c) 2010, 
    @author Stephane Embarki
   
    JRL-Japan, CNRS/AIST
    
    See license file for information on license.
*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

#include <iostream>

//#include <windows.h>
//#include <winbase.h>


//#if defined (VISP_HAVE_DIRECTSHOW) 
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9))




#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vp1394TwoGrabber.h> 

#include <visp/vpPixelMeterConversion.h>

#include <visp/vpPose.h>
#include <visp/vpDot2.h>
#include <visp/vpImagePoint.h>

// List of allowed command line options
#define GETOPTARGS  "di:p:hf:g:n:s:l:c"


using namespace std;

vpHomogeneousMatrix 
computePose_cMo(  vpCameraParameters & cam,
                  vpImage<unsigned char>& I,
                  const int nbpts,
                  vpPoint * Ptarget, vpDot2* d )
{


  for (int i=0 ; i < nbpts ; i++)
    {
	  // by using setGraphics, we request to see the all the pixel of the dot
	  // in green on the screen.
	  // It uses the overlay image plane.
	  // The default of this setting is that it is time consumming
      d[i].setGraphics(true);
    }
  // dot coordinates (u,v) = (column,row)
  cout << "Click the white points on the object corner anticlockwise in the 1394 camera view" <<endl  ;
  //double * u = new double [nbpts];
  //double * v = new double [nbpts];
  vpImagePoint vpIP[nbpts];


 try{
    for (int i=0 ; i < nbpts ; i++)
      {
	// tracking is initalized
	// if no other parameters are given to the iniTracking(..) method
	// a right mouse click on the dot is expected
	// dot location can also be specified explicitely in the initTracking
	// method  : d.initTracking(I,u,v)  where u is the column index and v is
	// the row index
	
	d[i].initTracking( I ) ;
	// track the dot and returns its coordinates in the image
	// results are given in float since many many are usually considered
	//
	// an expcetion is thrown by the track method if
	//  - dot is lost
	//  - the number of pixel is too small
	//  - too many pixels are detected (this is usual when a "big" specularity
	//    occurs. The threshold can be modified using the
	//    setNbMaxPoint(int) method
	

	d[i].track( I, vpIP[i] );

	vpDisplay :: flush(I);
      }
    
  }
  catch(...)
    {
      vpERROR_TRACE("Error in tracking initialization ") ;
      throw ;
    }
  
  for (int i=0 ; i < nbpts ; i++)
    vpDisplay::displayCross(I,vpIP[i] ,10,vpColor::red) ;
  
  // flush the X11 buffer
  vpDisplay::flush(I) ;
  
  

  ////////////////////////////////////////////////////////////
  //
      //Now we compute the pose of both camera
  //
  ////////////////////////////////////////////////////////////


  // We need a structure that content both the 3D coordinates of the point
  // in the object frame and the 2D coordinates of the point expressed in meter
  // the vpPoint class is ok for that


  


  // pixel-> meter conversion
  for (int i=0 ; i < nbpts ; i++)
    {
      // u[i]. v[i] are expressed in pixel
      // conversion in meter is achieved using
      // x = (u-u0)/px
      // y = (v-v0)/py
      // where px, py, u0, v0 are the intrinsic camera parameters
      double x,y ;
      vpPixelMeterConversion::convertPoint( cam ,vpIP[i] , x , y )  ;
      Ptarget[i].set_x(x) ;
      Ptarget[i].set_y(y) ;
      
      
    }
  
  
  // -----------------------------------------------------------------------
  // The pose structure is built, we put in the point list the set of point
  // here both 2D and 3D world coordinates are known
  // -----------------------------------------------------------------------
  // The vpPose class mainly contents a list of vpPoint (that is (X,Y,Z, x, y) )
  vpPose pose ;
  //  the list of point is cleared (if that's not done before)
  
  pose.clearPoint() ;
  for (int i= 0 ; i < nbpts ; i++)
    {
      pose.addPoint( Ptarget[ i ] ) ; // and added to the pose computation point list
      Ptarget[i].print();
    }
  
  // compute the initial pose using Dementhon method followed by a non linear
  // minimisation method
  
  
  // Pose by Lagrange it provides an initialization of the pose
  vpHomogeneousMatrix cMo;
  cout << " pose.computePose(vpPose::LAGRANGE, cMo ) ;"<<endl;
  pose.computePose(vpPose::LAGRANGE, cMo ) ;
  
  cout << pose.computeResidual( cMo ) <<endl ;
  
  // the pose is now refined using the virtual visual servoing approach
  // Warning: cMo needs to be initialized otherwise it may  diverge
  pose.computePose( vpPose::LOWE, cMo ) ;
  
  // display the compute pose
  pose.display( I , cMo , cam , 0.05 , vpColor::red ) ;
  vpDisplay::flush(I) ;
  
  cout << pose.computeResidual( cMo) <<endl;
 

  return cMo;
  

}

int main()
{
  
  // creation d une image en niveau de gris
  vpImage<unsigned char> I ;
  
  //creer un grabber pour communiquer avec la webcam
  vp1394TwoGrabber g  ;
  
  // displays pour l'affichage
  vpDisplayX d;
  
  // on recupere directement ces information du grabber
  vpTRACE(" grab the images");
  try{
    
    // on etablit la connexion
     g.open(I);
    // acquire une image
    //get image
    
    // acquiert depuis la web cam et stocke dans I 
    g.acquire(I);
    
    // set the display properties
    char title[50];
    sprintf(title,"Image");
    d.init(I,100,100,title); 
    
    // affichage : dispolay + flush
    
    vpDisplay::display(I);
    vpDisplay::flush(I);
    
    
  }
  catch(...){
    vpTRACE ( "Cannot acquire an image... " ) ;
    exit(-1);
    
  }
  
  
  // display all images till the user click on the image 0 
  std::cout << "Clic on the first view to start the acquisition, another click will stop it"<<std::endl;
  //getClick bloquant
  vpDisplay::getClick(I);
  
  vpTRACE("acquire many images");
  bool stopFlag = false;
  while (!stopFlag)
    {
      try{
	g.acquire(I);
	vpDisplay::display(I);
	vpDisplay::flush(I);
      }catch(...){
	
	vpTRACE ( "Cannot acquire the images... " ) ;
	exit(-1);
      }
      
      //getClick non bloquant : on arrete des que l'utilisateur clic
      stopFlag = vpDisplay::getClick(I,false);
      
    }
  stopFlag = false;
  
  // ------------------------------------------------------//
  // camera intrinsic parameters
  // ----------------------------------------------------//
  double px = 605.066347;
  double py = 605.4534825;
  double u0 = 313.4837733;       
  double v0 = 260.8029432;
  vpCameraParameters cam(px,py,u0,v0) ;
  
  // ------------------------------------------------------//
  // target description
  // ----------------------------------------------------//
  
  int nbpts(6);
  cout << "P2 \t \t \t P1"<< endl << endl <<endl ;
  cout << " \t \t P5"<< endl << endl <<endl ;
  cout << " P3 \t \t \t P4"<< endl << endl ;
  // We need a structure that content both the 3D coordinates of the point
  // in the object frame and the 2D coordinates of the point expressed in meter
  // the vpPoint class is ok for that
  vpPoint *  P_Target = new vpPoint [ nbpts ] ;
  // we set the 3D points coordinates (in meter !) in the object/world frame
  P_Target[0].setWorldCoordinates( -0.1, -0.05 , 0 ) ; // (X,Y,Z)
  P_Target[1].setWorldCoordinates( 0.02 , -0.06 , 0 ) ;
  P_Target[2].setWorldCoordinates( 0.05 , 0.07 , 0 ) ;
  P_Target[3].setWorldCoordinates( -0.1 , 0.06 , 0 ) ;
  P_Target[4].setWorldCoordinates( 0.1 , -0.02 ,  0 ) ;
  P_Target[5].setWorldCoordinates( 0 , 0 ,  0 ) ;
  
  // define the vpDot structure, here 5 dots will tracked
  vpDot2 * points2D = new vpDot2[nbpts] ;
  vpHomogeneousMatrix cMo ;
  cMo = computePose_cMo(  cam, I, nbpts, P_Target, points2D);
  
  
  // une fois le point clicke, on peut redemarrer l'acquisition en boucle
  std::cout << "Clic on the first view to launch the tracking \nanother click will stop it"<<std::endl;
  //getClick bloquant
  vpDisplay::getClick(I);
  
  //coord pîxelique
  vpImagePoint vpIP;
    //double u,v;

  vpPose pose ;
  //  the list of point is cleared (if that's not done before)
  pose.clearPoint() ;
  cout << "Tx \t Ty \t Tz "<<endl;
  
  while (!stopFlag)
    {
      pose.clearPoint() ;
      try{
	g.acquire(I);
	vpDisplay::display(I);
	vpDisplay::flush(I);
	
	for (int i = 0 ; i < nbpts ; i++){
	  points2D[i].track( I, vpIP ) ;
	  vpDisplay::displayCross(I,vpIP ,10,vpColor::red) ;
	  double x,y ;
	  vpPixelMeterConversion::convertPoint( cam ,vpIP , x , y )  ;
	  P_Target[i].set_x(x) ;
	  P_Target[i].set_y(y) ;
	  pose.addPoint( P_Target[ i ] ) ; // and added to the pose computation point list
	  
	}
	
	vpDisplay::flush(I);
	
	
	
      }catch(...){
	
	vpTRACE ( "Cannot acquire the images... " ) ;
	exit(-1);
      }
      
      
      cout << " pose.computePose(vpPose::LAGRANGE, cMo ) ;"<<endl;
      pose.computePose(vpPose::LAGRANGE, cMo ) ;
      cout << pose.computeResidual( cMo ) <<endl ;
      
      // the pose is now refined using the virtual visual servoing approach
      // Warning: cMo needs to be initialized otherwise it may  diverge
      pose.computePose( vpPose::LOWE, cMo ) ;
      
      // display the compute pose
      pose.display( I , cMo , cam , 0.05 , vpColor::red ) ;
      vpDisplay::flush(I) ;
      
      
      cout <<  cMo[0][3] << "\t "<< cMo[1][3] << "\t " << cMo[2][3]  <<endl;
      
      //getClick non bloquant
      stopFlag = vpDisplay::getClick(I,false);
      
    }

  


  
  // wait for the user action
  std::cout << "Clic on the first view to close the application"<<std::endl;
  vpDisplay::getClick(I);
  
		
  // end of main
  delete [] P_Target;
  delete [] points2D;
  return(0);
}


#endif // (defined (VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)...)
//#endif 
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
