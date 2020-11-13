#include "RdvFindOrangeWithRealsense.h"

#if 0

int main(int argc, char * argv[])
{
	printf("Rendezvue Orange Detection\n") ;

	CRdvFindOrangeWithRealsense cls_rdv_find_orange_w_realsense("../train/iforange.cfg", "../train/iforange_last.weights", "../train/iforange.data") ;

	Orange orange = cls_rdv_find_orange_w_realsense.Run(true) ;

	printf("orange : z(%f, %f, %f) meter, w(%f, %f, %f) meter, (%f, %f, %f) degree, %f\n", orange.center_x, orange.center_y, orange.center_z, orange.wcenter_x, orange.wcenter_y, orange.wcenter_z, orange.rotate_x, orange.rotate_y, orange.rotate_z, orange.size ) ;
	
    return EXIT_SUCCESS;
}
#endif
