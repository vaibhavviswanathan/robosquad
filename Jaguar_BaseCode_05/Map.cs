using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class Map
    {
        public int numMapSegments = 0;
        public double[,,] mapSegmentCorners;
        public double minX, maxX, minY, maxY;
        private double[] slopes;
        private double[] segmentSizes;
        private double[] intercepts;

        private double minWorkspaceX = -50;
        private double maxWorkspaceX =  50;
        private double minWorkspaceY = -50;
        private double maxWorkspaceY =  50;

        public Map()
        {

	        // This is hard coding at its worst. Just edit the file to put in
	        // segments of the environment your robot is working in. This is
	        // used both for visual display and for localization.

	        // ****************** Additional Student Code: Start ************
	
	        // Change hard code here to change map:

	        numMapSegments = 122;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];
/*
            mapSegmentCorners[0, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
	        mapSegmentCorners[0,0,1] = 2.794;
            mapSegmentCorners[0, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[0, 1, 1] = 2.794;

	        mapSegmentCorners[1,0,0] = -3.55/2;
	        mapSegmentCorners[1,0,1] = 0.0;
	        mapSegmentCorners[1,1,0] = -3.55/2;
	        mapSegmentCorners[1,1,1] = -2.74;

	        mapSegmentCorners[2,0,0] = 3.55/2;
	        mapSegmentCorners[2,0,1] = 0.0;
	        mapSegmentCorners[2,1,0] = 3.55/2;
	        mapSegmentCorners[2,1,1] = -2.74;

            mapSegmentCorners[3, 0, 0] = 3.55/2;
            mapSegmentCorners[3, 0, 1] = 0.0;
            mapSegmentCorners[3, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[3, 1, 1] = 0.0;

            mapSegmentCorners[4, 0, 0] = -3.55/2;
            mapSegmentCorners[4, 0, 1] = 0.0;
            mapSegmentCorners[4, 1, 0] = -3.55/2 - 5.79;
            mapSegmentCorners[4, 1, 1] = 0.0;

            mapSegmentCorners[5, 0, 0] = -3.55/2;
            mapSegmentCorners[5, 0, 1] = -2.74;
            mapSegmentCorners[5, 1, 0] = -3.55/2-3.05;
            mapSegmentCorners[5, 1, 1] = -2.74;

            mapSegmentCorners[6, 0, 0] = 3.55 / 2;
            mapSegmentCorners[6, 0, 1] = -2.74;
            mapSegmentCorners[6, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[6, 1, 1] = -2.74;

            mapSegmentCorners[7, 0, 0] = 5.03 / 2;
            mapSegmentCorners[7, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[7, 1, 0] = -5.03/2;
            mapSegmentCorners[7, 1, 1] = -2.74 - 2.31;
*/

//top area walls	
mapSegmentCorners[0,0,0] = -5.04 / 2	;
mapSegmentCorners[0,0,1] = 2.36	;
mapSegmentCorners[0,1,0] = -5.04 / 2	;
mapSegmentCorners[0,1,1] = 2.36 + 5.03	;
	
mapSegmentCorners[1,0,0] = 5.04 / 2	;
mapSegmentCorners[1,0,1] = 2.36	;
mapSegmentCorners[1,1,0] = 5.04 / 2	;
mapSegmentCorners[1,1,1] = 2.36 + 5.03	;
	
mapSegmentCorners[2,0,0] = 5.04 / 2	;
mapSegmentCorners[2,0,1] = 2.36	;
mapSegmentCorners[2,1,0] = -5.04 / 2	;
mapSegmentCorners[2,1,1] = 2.36	;
	
mapSegmentCorners[3,0,0] = -4.82	;
mapSegmentCorners[3,0,1] = 0.0	;
mapSegmentCorners[3,1,0] = -4.82	;
mapSegmentCorners[3,1,1] = 9.58	;
	
mapSegmentCorners[4,0,0] = 4.82	;
mapSegmentCorners[4,0,1] = 0.0	;
mapSegmentCorners[4,1,0] = 4.82	;
mapSegmentCorners[4,1,1] = 9.58	;
	
mapSegmentCorners[5,0,0] = -1.77	;
mapSegmentCorners[5,0,1] = 0.0	;
mapSegmentCorners[5,1,0] = -4.82	;
mapSegmentCorners[5,1,1] = 0.0	;
	
mapSegmentCorners[6,0,0] = 1.77	;
mapSegmentCorners[6,0,1] = 0.0	;
mapSegmentCorners[6,1,0] = 4.82	;
mapSegmentCorners[6,1,1] = 0.0	;
	
mapSegmentCorners[7,0,0] = -1.77	;
mapSegmentCorners[7,0,1] = 0.0	;
mapSegmentCorners[7,1,0] = -1.77	;
mapSegmentCorners[7,1,1] = -2.74	;
	
mapSegmentCorners[8,0,0] = 1.77	;
mapSegmentCorners[8,0,1] = 0.0	;
mapSegmentCorners[8,1,0] = 1.77	;
mapSegmentCorners[8,1,1] = -2.74	;
	
mapSegmentCorners[9,0,0] = -1.77	;
mapSegmentCorners[9,0,1] = -2.74	;
mapSegmentCorners[9,1,0] = -1.77 - 5.80	;
mapSegmentCorners[9,1,1] = -2.74	;
	
mapSegmentCorners[10,0,0] = 1.77	;
mapSegmentCorners[10,0,1] = -2.74	;
mapSegmentCorners[10,1,0] = 1.77 + 5.80	;
mapSegmentCorners[10,1,1] = -2.74	;
	
mapSegmentCorners[108,0,0] = 1.77 + 5.80	;
mapSegmentCorners[108,0,1] = -2.74	;
mapSegmentCorners[108,1,0] = 1.77 + 5.80	;
mapSegmentCorners[108,1,1] = 2.36 + 2.0	;
	
mapSegmentCorners[109,0,0] = -5.04 / 2	;
mapSegmentCorners[109,0,1] = 2.36 + 5.03	;
mapSegmentCorners[109,1,0] = 5.04 / 2	;
mapSegmentCorners[109,1,1] = 2.36 + 5.03	;
	
mapSegmentCorners[110,0,0] = -1.77	;
mapSegmentCorners[110,0,1] = 9.58	;
mapSegmentCorners[110,1,0] = -1.77	;
mapSegmentCorners[110,1,1] = 9.58 + 2.74	;
	
mapSegmentCorners[111,0,0] = 1.77	;
mapSegmentCorners[111,0,1] = 9.58	;
mapSegmentCorners[111,1,0] = 1.77	;
mapSegmentCorners[111,1,1] = 9.58 + 2.74	;
	
mapSegmentCorners[112,0,0] = -4.82	;
mapSegmentCorners[112,0,1] = 9.58	;
mapSegmentCorners[112,1,0] = -1.77	;
mapSegmentCorners[112,1,1] = 9.58	;
	
mapSegmentCorners[113,0,0] = 4.82	;
mapSegmentCorners[113,0,1] = 9.58	;
mapSegmentCorners[113,1,0] = 1.77	;
mapSegmentCorners[113,1,1] = 9.58	;
	
//bottom area walls	
mapSegmentCorners[11,0,0] = -1.77	;
mapSegmentCorners[11,0,1] = -19.82	;
mapSegmentCorners[11,1,0] = -1.77 - 1.82 - 3.97	;
mapSegmentCorners[11,1,1] = -19.82	;
	
mapSegmentCorners[12,0,0] = 1.77	;
mapSegmentCorners[12,0,1] = -19.82	;
mapSegmentCorners[12,1,0] = 14.86	;
mapSegmentCorners[12,1,1] = -19.82	;
	
mapSegmentCorners[13,0,0] = -1.77	;
mapSegmentCorners[13,0,1] = -19.82	;
mapSegmentCorners[13,1,0] = -1.77	;
mapSegmentCorners[13,1,1] = -23.47	;
	
mapSegmentCorners[14,0,0] = 1.77	;
mapSegmentCorners[14,0,1] = -19.82	;
mapSegmentCorners[14,1,0] = 1.77	;
mapSegmentCorners[14,1,1] = -23.47	;
	
mapSegmentCorners[15,0,0] = -1.77	;
mapSegmentCorners[15,0,1] = -23.47	;
mapSegmentCorners[15,1,0] = -1.77 - 1.82	;
mapSegmentCorners[15,1,1] = -23.47	;
	
mapSegmentCorners[16,0,0] = 1.77	;
mapSegmentCorners[16,0,1] = -23.47	;
mapSegmentCorners[16,1,0] = 10.90	;
mapSegmentCorners[16,1,1] = -23.47	;
	
mapSegmentCorners[17,0,0] = -1.77 - 1.82	;
mapSegmentCorners[17,0,1] = -23.47	;
mapSegmentCorners[17,1,0] = -1.77 - 1.82	;
mapSegmentCorners[17,1,1] = -23.47 - 3.50	;
	
mapSegmentCorners[18,0,0] = 10.90	;
mapSegmentCorners[18,0,1] = -23.47	;
mapSegmentCorners[18,1,0] = 10.90	;
mapSegmentCorners[18,1,1] = -23.47 - 3.50	;
	
mapSegmentCorners[19,0,0] = -1.77 - 1.82 - 3.97	;
mapSegmentCorners[19,0,1] = -19.82	;
mapSegmentCorners[19,1,0] = -1.77 - 1.82 - 3.97	;
mapSegmentCorners[19,1,1] = -23.47 - 3.50	;
	
mapSegmentCorners[20,0,0] = 14.86	;
mapSegmentCorners[20,0,1] = -19.82	;
mapSegmentCorners[20,1,0] = 14.86	;
mapSegmentCorners[20,1,1] = -23.47 - 3.50	;
	
mapSegmentCorners[21,0,0] = -1.77 - 1.82	;
mapSegmentCorners[21,0,1] = -23.47 - 3.50	;
mapSegmentCorners[21,1,0] = -1.77 - 1.82 - 3.97	;
mapSegmentCorners[21,1,1] = -23.47 - 3.50	;
	
mapSegmentCorners[22,0,0] = 10.90	;
mapSegmentCorners[22,0,1] = -23.47 - 3.50	;
mapSegmentCorners[22,1,0] = 14.86	;
mapSegmentCorners[22,1,1] = -23.47 - 3.50	;
	
mapSegmentCorners[23,0,0] = -1.77 - 1.82	;
mapSegmentCorners[23,0,1] = -23.47 - 3.50 - 2.90	;
mapSegmentCorners[23,1,0] = -1.77 - 1.82 - 3.97	;
mapSegmentCorners[23,1,1] = -23.47 - 3.50 - 2.90	;
	
mapSegmentCorners[24,0,0] = 10.90	;
mapSegmentCorners[24,0,1] = -23.47 - 3.50 - 2.90	;
mapSegmentCorners[24,1,0] = 14.86	;
mapSegmentCorners[24,1,1] = -23.47 - 3.50 - 2.90	;
	
mapSegmentCorners[25,0,0] = -1.77 - 1.82	;
mapSegmentCorners[25,0,1] = -23.47 - 3.50 - 2.90	;
mapSegmentCorners[25,1,0] = -1.77 - 1.82	;
mapSegmentCorners[25,1,1] = -33.37	;
	
mapSegmentCorners[26,0,0] = 10.90	;
mapSegmentCorners[26,0,1] = -23.47 - 3.50 - 2.90	;
mapSegmentCorners[26,1,0] = 10.90	;
mapSegmentCorners[26,1,1] = -33.37	;
	
mapSegmentCorners[27,0,0] = -1.77	;
mapSegmentCorners[27,0,1] = -33.37	;
mapSegmentCorners[27,1,0] = -1.77 - 1.82	;
mapSegmentCorners[27,1,1] = -33.37	;
	
mapSegmentCorners[28,0,0] = 1.77	;
mapSegmentCorners[28,0,1] = -33.37	;
mapSegmentCorners[28,1,0] = 10.90	;
mapSegmentCorners[28,1,1] = -33.37	;
	
mapSegmentCorners[29,0,0] = -1.77	;
mapSegmentCorners[29,0,1] = -33.37	;
mapSegmentCorners[29,1,0] = -1.77	;
mapSegmentCorners[29,1,1] = -37.02	;
	
mapSegmentCorners[30,0,0] = 1.77	;
mapSegmentCorners[30,0,1] = -33.37	;
mapSegmentCorners[30,1,0] = 1.77	;
mapSegmentCorners[30,1,1] = -37.02	;
	
//pillars	
mapSegmentCorners[31,0,0] = -3.35 - 0.34/2	;
mapSegmentCorners[31,0,1] = -9.63	;
mapSegmentCorners[31,1,0] = -3.35 - 0.34/2 - 0.34	;
mapSegmentCorners[31,1,1] = -9.63	;
	
mapSegmentCorners[32,0,0] = -3.35 - 0.34/2 - 0.34	;
mapSegmentCorners[32,0,1] = -9.63 - 0.29	;
mapSegmentCorners[32,1,0] = -3.35 - 0.34/2 - 0.34	;
mapSegmentCorners[32,1,1] = -9.63	;
	
mapSegmentCorners[33,0,0] = -3.35 - 0.34/2 - 0.34	;
mapSegmentCorners[33,0,1] = -9.63 - 0.29	;
mapSegmentCorners[33,1,0] = -3.35 - 0.34/2	;
mapSegmentCorners[33,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[34,0,0] = -3.35 - 0.34/2	;
mapSegmentCorners[34,0,1] = -9.63	;
mapSegmentCorners[34,1,0] = -3.35 - 0.34/2	;
mapSegmentCorners[34,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[35,0,0] = -0.34/2	;
mapSegmentCorners[35,0,1] = -9.63	;
mapSegmentCorners[35,1,0] = 0.34/2	;
mapSegmentCorners[35,1,1] = -9.63	;
	
mapSegmentCorners[36,0,0] = -0.34/2	;
mapSegmentCorners[36,0,1] = -9.63 - 0.29	;
mapSegmentCorners[36,1,0] = -0.34/2	;
mapSegmentCorners[36,1,1] = -9.63	;
	
mapSegmentCorners[37,0,0] = -0.34/2	;
mapSegmentCorners[37,0,1] = -9.63 - 0.29	;
mapSegmentCorners[37,1,0] = 0.34/2	;
mapSegmentCorners[37,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[38,0,0] = 0.34/2	;
mapSegmentCorners[38,0,1] = -9.63	;
mapSegmentCorners[38,1,0] = 0.34/2	;
mapSegmentCorners[38,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[39,0,0] = 3.35 + 0.34/2	;
mapSegmentCorners[39,0,1] = -9.63	;
mapSegmentCorners[39,1,0] = 3.35 + 0.34/2 + 0.34	;
mapSegmentCorners[39,1,1] = -9.63	;
	
mapSegmentCorners[40,0,0] = 3.35 + 0.34/2	;
mapSegmentCorners[40,0,1] = -9.63 - 0.29	;
mapSegmentCorners[40,1,0] = 3.35 + 0.34/2	;
mapSegmentCorners[40,1,1] = -9.63	;
	
mapSegmentCorners[41,0,0] = 3.35 + 0.34/2	;
mapSegmentCorners[41,0,1] = -9.63 - 0.29	;
mapSegmentCorners[41,1,0] = 3.35 + 0.34/2 + 0.34	;
mapSegmentCorners[41,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[42,0,0] = 3.35 + 0.34/2 + 0.34	;
mapSegmentCorners[42,0,1] = -9.63	;
mapSegmentCorners[42,1,0] = 3.35 + 0.34/2 + 0.34	;
mapSegmentCorners[42,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[43,0,0] = -3.35 - 0.34/2	;
mapSegmentCorners[43,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[43,1,0] = -3.35 - 0.34/2 - 0.34	;
mapSegmentCorners[43,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[44,0,0] = -3.35 - 0.34/2 - 0.34	;
mapSegmentCorners[44,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[44,1,0] = -3.35 - 0.34/2 - 0.34	;
mapSegmentCorners[44,1,1] = -9.63 - 0.29 - 3.16 - 0.29	;
	
mapSegmentCorners[45,0,0] = -3.35 - 0.34/2 - 0.34	;
mapSegmentCorners[45,0,1] = -9.63 - 0.29 - 3.16 - 0.29	;
mapSegmentCorners[45,1,0] = -3.35 - 0.34/2	;
mapSegmentCorners[45,1,1] = -9.63 - 0.29 - 3.16 - 0.29	;
	
mapSegmentCorners[46,0,0] = -3.35 - 0.34/2	;
mapSegmentCorners[46,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[46,1,0] = -3.35 - 0.34/2	;
mapSegmentCorners[46,1,1] = -9.63 - 0.29 - 3.16 - 0.29	;
	
mapSegmentCorners[47,0,0] = -0.34/2	;
mapSegmentCorners[47,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[47,1,0] = 0.34/2	;
mapSegmentCorners[47,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[48,0,0] = -0.34/2	;
mapSegmentCorners[48,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[48,1,0] = -0.34/2	;
mapSegmentCorners[48,1,1] = -9.63 - 0.29 - 3.16 - 0.29	;
	
mapSegmentCorners[49,0,0] = -0.34/2	;
mapSegmentCorners[49,0,1] = -9.63 - 0.29 - 3.16 - 0.29	;
mapSegmentCorners[49,1,0] = 0.34/2	;
mapSegmentCorners[49,1,1] = -9.63 - 0.29 - 3.16 - 0.29	;
	
mapSegmentCorners[50,0,0] = 0.34/2	;
mapSegmentCorners[50,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[50,1,0] = 0.34/2	;
mapSegmentCorners[50,1,1] = -9.63 - 0.29 - 3.16 - 0.29	;
	
mapSegmentCorners[51,0,0] = 3.35 + 0.34/2	;
mapSegmentCorners[51,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[51,1,0] = 3.35 + 0.34/2 + 0.34	;
mapSegmentCorners[51,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[52,0,0] = 3.35 + 0.34/2	;
mapSegmentCorners[52,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[52,1,0] = 3.35 + 0.34/2	;
mapSegmentCorners[52,1,1] = -9.63 - 0.29 - 3.16 - 0.29	;
	
mapSegmentCorners[53,0,0] = 3.35 + 0.34/2	;
mapSegmentCorners[53,0,1] = -9.63 - 0.29 - 3.16 - 0.29	;
mapSegmentCorners[53,1,0] = 3.35 + 0.34/2 + 0.34	;
mapSegmentCorners[53,1,1] = -9.63 - 0.29 - 3.16 - 0.29	;
	
mapSegmentCorners[54,0,0] = 3.35 + 0.34/2 + 0.34	;
mapSegmentCorners[54,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[54,1,0] = 3.35 + 0.34/2 + 0.34	;
mapSegmentCorners[54,1,1] = -9.63 - 0.29 - 3.16 - 0.29	;
	
mapSegmentCorners[55,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[55,0,1] = -9.63	;
mapSegmentCorners[55,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[55,1,1] = -9.63	;
	
mapSegmentCorners[56,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[56,0,1] = -9.63 - 0.29	;
mapSegmentCorners[56,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[56,1,1] = -9.63	;
	
mapSegmentCorners[57,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[57,0,1] = -9.63 - 0.29	;
mapSegmentCorners[57,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[57,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[58,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[58,0,1] = -9.63	;
mapSegmentCorners[58,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[58,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[59,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[59,0,1] = -9.63	;
mapSegmentCorners[59,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[59,1,1] = -9.63	;
	
mapSegmentCorners[60,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 	;
mapSegmentCorners[60,0,1] = -9.63 - 0.29	;
mapSegmentCorners[60,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[60,1,1] = -9.63	;
	
mapSegmentCorners[61,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[61,0,1] = -9.63 - 0.29	;
mapSegmentCorners[61,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[61,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[62,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[62,0,1] = -9.63	;
mapSegmentCorners[62,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[62,1,1] = -9.63 - 0.29        	;
	
mapSegmentCorners[63,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[63,0,1] = -9.63	;
mapSegmentCorners[63,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[63,1,1] = -9.63	;
	
mapSegmentCorners[64,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 	;
mapSegmentCorners[64,0,1] = -9.63 - 0.29	;
mapSegmentCorners[64,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[64,1,1] = -9.63	;
	
mapSegmentCorners[65,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[65,0,1] = -9.63 - 0.29	;
mapSegmentCorners[65,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[65,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[66,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[66,0,1] = -9.63	;
mapSegmentCorners[66,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[66,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[67,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[67,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[67,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[67,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[68,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[68,0,1] = -9.63 - 0.29 - 3.16 - 0.29	;
mapSegmentCorners[68,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[68,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[69,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[69,0,1] = -9.63 - 0.29 - 0.29 - 3.16	;
mapSegmentCorners[69,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[69,1,1] = -9.63 - 0.29 - 0.29 - 3.16	;
	
mapSegmentCorners[70,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[70,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[70,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[70,1,1] = -9.63 - 0.29 - 0.29 - 3.16	;
	
mapSegmentCorners[71,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[71,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[71,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[71,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[72,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 	;
mapSegmentCorners[72,0,1] = -9.63 - 0.29 - 0.29 - 3.16	;
mapSegmentCorners[72,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[72,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[73,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[73,0,1] = -9.63 - 0.29 - 0.29 - 3.16	;
mapSegmentCorners[73,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[73,1,1] = -9.63 - 0.29 - 0.29 - 3.16	;
	
mapSegmentCorners[74,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[74,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[74,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[74,1,1] = -9.63 - 0.29 - 0.29 - 3.16 	;
	
mapSegmentCorners[75,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[75,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[75,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[75,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[76,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 	;
mapSegmentCorners[76,0,1] = -9.63 - 0.29 - 0.29 - 3.16	;
mapSegmentCorners[76,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[76,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[77,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[77,0,1] = -9.63 - 0.29 - 0.29 - 3.16	;
mapSegmentCorners[77,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[77,1,1] = -9.63 - 0.29 - 0.29 - 3.16	;
	
mapSegmentCorners[78,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[78,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[78,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[78,1,1] = -9.63 - 0.29 - 0.29 - 3.16	;
	
mapSegmentCorners[79,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[79,0,1] = -9.63	;
mapSegmentCorners[79,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[79,1,1] = -9.63	;
	
mapSegmentCorners[80,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[80,0,1] = -9.63 - 0.29	;
mapSegmentCorners[80,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[80,1,1] = -9.63	;
	
mapSegmentCorners[81,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[81,0,1] = -9.63 - 0.29	;
mapSegmentCorners[81,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[81,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[82,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[82,0,1] = -9.63	;
mapSegmentCorners[82,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[82,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[100,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[100,0,1] = -9.63	;
mapSegmentCorners[100,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[100,1,1] = -9.63	;
	
mapSegmentCorners[101,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[101,0,1] = -9.63 - 0.29	;
mapSegmentCorners[101,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[101,1,1] = -9.63	;
	
mapSegmentCorners[102,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35	;
mapSegmentCorners[102,0,1] = -9.63 - 0.29	;
mapSegmentCorners[102,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[102,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[103,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[103,0,1] = -9.63	;
mapSegmentCorners[103,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[103,1,1] = -9.63 - 0.29	;
	
mapSegmentCorners[104,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[104,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[104,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[104,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[105,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[105,0,1] = -9.63 - 0.29 - 0.29 - 3.16	;
mapSegmentCorners[105,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[105,1,1] = -9.63 - 0.29 - 3.16	;
	
mapSegmentCorners[106,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[106,0,1] = -9.63 - 0.29 - 0.29 - 3.16	;
mapSegmentCorners[106,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[106,1,1] = -9.63 - 0.29 - 0.29 - 3.16	;
	
mapSegmentCorners[107,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[107,0,1] = -9.63 - 0.29 - 3.16	;
mapSegmentCorners[107,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[107,1,1] = -9.63 - 0.29 - 0.29 - 3.16	;
	
mapSegmentCorners[114,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[114,0,1] = -9.63 - 0.29 - 3.16 - 0.29 - 3.16	;
mapSegmentCorners[114,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[114,1,1] = -9.63 - 0.29 - 3.16 - 0.29 - 3.16	;
	
mapSegmentCorners[115,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[115,0,1] = -9.63 - 0.29 - 0.29 - 3.16 - 0.29 - 3.16	;
mapSegmentCorners[115,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[115,1,1] = -9.63 - 0.29 - 3.16 - 0.29 - 3.16	;
	
mapSegmentCorners[116,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[116,0,1] = -9.63 - 0.29 - 0.29 - 3.16 - 0.29 - 3.16	;
mapSegmentCorners[116,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[116,1,1] = -9.63 - 0.29 - 0.29 - 3.16 - 0.29 - 3.16	;
	
mapSegmentCorners[117,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[117,0,1] = -9.63 - 0.29 - 3.16 - 0.29 - 3.16	;
mapSegmentCorners[117,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[117,1,1] = -9.63 - 0.29 - 0.29 - 3.16 - 0.29 - 3.16	;
	
mapSegmentCorners[118,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[118,0,1] = -9.63 - 0.29 - 3.16 - 0.29 - 3.16 - 0.29 - 3.16	;
mapSegmentCorners[118,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[118,1,1] = -9.63 - 0.29 - 3.16 - 0.29 - 3.16 - 0.29 - 3.16	;
	
mapSegmentCorners[119,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[119,0,1] = -9.63 - 0.29 - 0.29 - 3.16 - 0.29 - 3.16 - 0.29 - 3.16	;
mapSegmentCorners[119,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[119,1,1] = -9.63 - 0.29 - 3.16 - 0.29 - 3.16 - 0.29 - 3.16	;
	
mapSegmentCorners[120,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 3.35 + 0.34	;
mapSegmentCorners[120,0,1] = -9.63 - 0.29 - 0.29 - 3.16 - 0.29 - 3.16 - 0.29 - 3.16	;
mapSegmentCorners[120,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[120,1,1] = -9.63 - 0.29 - 0.29 - 3.16 - 0.29 - 3.16 - 0.29 - 3.16	;
	
mapSegmentCorners[121,0,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[121,0,1] = -9.63 - 0.29 - 3.16 - 0.29 - 3.16 - 0.29 - 3.16	;
mapSegmentCorners[121,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34	;
mapSegmentCorners[121,1,1] = -9.63 - 0.29 - 0.29 - 3.16 - 0.29 - 3.16 - 0.29 - 3.16	;
	
// extra walls	;
// mapSegmentCorners[104,1,0] = 0.34/2 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 + 3.35 + 0.34 = 18.62	;
// mapSegmentCorners[104,1,1] = -9.63 - 0.29 - 3.16 = -13.08	;
// choose the above point as new (0, 0) point when measuring	;
double x_offset = 18.62	;
double y_offset = -13.08	;
	
mapSegmentCorners[83,0,0] = x_offset - 7.90	;
mapSegmentCorners[83,0,1] = y_offset + 6.60	;
mapSegmentCorners[83,1,0] = x_offset - 7.90	;
mapSegmentCorners[83,1,1] = 2.36 + 2.0	;
	
mapSegmentCorners[84,0,0] = x_offset	;
mapSegmentCorners[84,0,1] = y_offset + 6.60	;
mapSegmentCorners[84,1,0] = x_offset - 7.90	;
mapSegmentCorners[84,1,1] = y_offset + 6.60	;
	
mapSegmentCorners[85,0,0] = x_offset	;
mapSegmentCorners[85,0,1] = y_offset + 6.60	;
mapSegmentCorners[85,1,0] = x_offset	;
mapSegmentCorners[85,1,1] = y_offset + 6.60 + 1.09	;
	
mapSegmentCorners[86,0,0] = x_offset	;
mapSegmentCorners[86,0,1] = y_offset + 6.60 + 1.09	;
mapSegmentCorners[86,1,0] = x_offset + 1.98	;
mapSegmentCorners[86,1,1] = y_offset + 6.60 + 1.09	;
	
mapSegmentCorners[87,0,0] = x_offset + 1.98	;
mapSegmentCorners[87,0,1] = y_offset + 6.60 + 1.09	;
mapSegmentCorners[87,1,0] = x_offset + 1.98	;
mapSegmentCorners[87,1,1] = y_offset + 6.60	;
	
mapSegmentCorners[88,0,0] = x_offset + 1.98	;
mapSegmentCorners[88,0,1] = y_offset + 6.60	;
mapSegmentCorners[88,1,0] = x_offset + 1.98 + 5.49	;
mapSegmentCorners[88,1,1] = y_offset + 6.60	;
	
mapSegmentCorners[89,0,0] = x_offset + 1.98 + 5.49	;
mapSegmentCorners[89,0,1] = y_offset + 6.60	;
mapSegmentCorners[89,1,0] = x_offset + 1.98 + 5.49	;
mapSegmentCorners[89,1,1] = 2.36 + 2.0	;
	
mapSegmentCorners[90,0,0] = x_offset + 10.11	;
mapSegmentCorners[90,0,1] = y_offset + 4.11	;
mapSegmentCorners[90,1,0] = x_offset + 10.11	;
mapSegmentCorners[90,1,1] = y_offset + 4.11 + 0.5	;
	
mapSegmentCorners[91,0,0] = x_offset + 10.11	;
mapSegmentCorners[91,0,1] = y_offset + 4.11	;
mapSegmentCorners[91,1,0] = x_offset + 10.11 + 1.45	;
mapSegmentCorners[91,1,1] = y_offset + 4.11	;
	
mapSegmentCorners[92,0,0] = x_offset + 10.11 + 1.45	;
mapSegmentCorners[92,0,1] = y_offset + 4.11	;
mapSegmentCorners[92,1,0] = x_offset + 10.11 + 1.45	;
mapSegmentCorners[92,1,1] = y_offset + 4.11 + 0.5	;
	
mapSegmentCorners[93,0,0] = x_offset + 3.35	;
mapSegmentCorners[93,0,1] = y_offset	;
mapSegmentCorners[93,1,0] = x_offset + 3.35	;
mapSegmentCorners[93,1,1] = -23.47 - 3.50	;
	
mapSegmentCorners[94,0,0] = x_offset + 3.35	;
mapSegmentCorners[94,0,1] = y_offset	;
mapSegmentCorners[94,1,0] = x_offset + 3.35 + 23.98/2	;
mapSegmentCorners[94,1,1] = y_offset	;
	
mapSegmentCorners[95,0,0] = x_offset + 3.35 + 23.98/2	;
mapSegmentCorners[95,0,1] = y_offset	;
mapSegmentCorners[95,1,0] = x_offset + 3.35 + 23.98	;
mapSegmentCorners[95,1,1] = y_offset	;
	
mapSegmentCorners[96,0,0] = x_offset + 3.35 + 23.98	;
mapSegmentCorners[96,0,1] = y_offset	;
mapSegmentCorners[96,1,0] = x_offset + 3.35 + 23.98	;
mapSegmentCorners[96,1,1] = y_offset - 6.71	;
	
mapSegmentCorners[97,0,0] = x_offset + 3.35 + 23.98	;
mapSegmentCorners[97,0,1] = y_offset - 6.71	;
mapSegmentCorners[97,1,0] = x_offset + 3.35 + 23.98 + 2.03	;
mapSegmentCorners[97,1,1] = y_offset - 6.71	;
	
mapSegmentCorners[98,0,0] = x_offset + 3.35 + 23.98 + 2.03	;
mapSegmentCorners[98,0,1] = y_offset - 6.71	;
mapSegmentCorners[98,1,0] = x_offset + 3.35 + 23.98 + 2.03	;
mapSegmentCorners[98,1,1] = y_offset - 6.71 - 3.75	;
	
mapSegmentCorners[99,0,0] = x_offset + 3.35 + 23.98 + 2.03	;
mapSegmentCorners[99,0,1] = y_offset - 6.71 - 3.75	;
mapSegmentCorners[99,1,0] = x_offset + 3.35 + 23.98 + 2.03 - 4.05	;
mapSegmentCorners[99,1,1] = y_offset - 6.71 - 3.75	;
	


            // ****************** Additional Student Code: End   ************


	        // Set map parameters
	        // These will be useful in your future coding.
	        minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
	        for (int i=0; i< numMapSegments; i++){
                // Offset map
                mapSegmentCorners[i, 0, 1] += 16;
                mapSegmentCorners[i, 1, 1] += 16;

                mapSegmentCorners[i, 0, 0] += -25;
                mapSegmentCorners[i, 1, 0] += -25;

                double scaling_factor = 1;

                mapSegmentCorners[i, 0, 1] *= scaling_factor;
                mapSegmentCorners[i, 1, 1] *= scaling_factor;

                mapSegmentCorners[i, 0, 0] *= scaling_factor;
                mapSegmentCorners[i, 1, 0] *= scaling_factor;



                // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
		
		        // Set wall segments to be horizontal
		        slopes[i] = (mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1])/(0.001+mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0]);
		        intercepts[i] = mapSegmentCorners[i,0,1] - slopes[i]*mapSegmentCorners[i,0,0];

		        // Set wall segment lengths
		        segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0],2)+Math.Pow(mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1],2));
	        }
        }


        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)
        double GetWallDistance(double x, double y, double t, int segment){

            double x1 = mapSegmentCorners[segment, 0, 0];
            double x2 = mapSegmentCorners[segment, 1, 0];

            double y1 = mapSegmentCorners[segment, 0, 1];
            double y2 = mapSegmentCorners[segment, 1, 1];

            if ((Math.Abs(x1 - x) >= 6.0 && Math.Abs(x2 - x) >= 6.0) || (Math.Abs(y1 - y) >= 6.0 && Math.Abs(y2 - y) >= 6.0))
                return 9999;

            double m_wall = slopes[segment];
            double b_wall = intercepts[segment];
            double size = segmentSizes[segment];

            double m_bot = Math.Tan(t);
            double b_bot = y - m_bot * x;

            double x_c = m_wall == m_bot ? 100000000000 : (b_bot - b_wall) / (m_wall - m_bot);
            double y_c = m_wall * x_c + b_wall;

            double tol = 0.001;

            bool validSeg = ((x_c >= Math.Min(x1, x2) - tol) && (x_c <= Math.Max(x1, x2) + tol)) && ((y_c >= Math.Min(y1, y2) - tol) && (y_c <= Math.Max(y1, y2) + tol));

            // check if in right heading of the robot
            validSeg &= (Math.Sign(t) == Math.Sign(y_c - y));

            double wallDist = validSeg ? Math.Sqrt(Math.Pow((x_c - x), 2) + Math.Pow((y_c - y), 2)) : Double.PositiveInfinity;
            //double wallDist = validSeg ? (Math.Pow((x_c - x), 2) + Math.Pow((y_c - y), 2)) : Double.PositiveInfinity;


            // ****************** Additional Student Code: End   ************

            return wallDist;

        }


        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        public double GetClosestWallDistance(double x, double y, double t){

            double minDist = 6.000;

            // ****************** Additional Student Code: Start ************

            // Put code here that loops through segments, calling the
            // function GetWallDistance.

            int i;
            for (i = 0; i < numMapSegments; i++)
            {
                double dist = GetWallDistance(x, y, t, i);
                if (dist < minDist) minDist = dist;
            }



            // ****************** Additional Student Code: End   ************

            return minDist;
        }


        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        public bool CollisionFound(Navigation.Node n1, Navigation.Node n2, double tol)
        {


            // Check that within boundaries
            if (n2.x > maxWorkspaceX || n2.x < minWorkspaceX || n2.y > maxWorkspaceY || n2.y < minWorkspaceY)
                return true;


            // Check for collision with walls
            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x);
            double edgeSize = Math.Sqrt(Math.Pow(n2.y - n1.y, 2) + Math.Pow(n2.x - n1.x, 2));
            double sinTheta = Math.Sin(theta);
            double cosTheta = Math.Cos(theta);

            // Loop through segments
            for (int segment = 0; segment < numMapSegments; segment++)
            {

                double distTravelledOnEdge = 0;
                double ex = n1.x, ey = n1.y;
                double distToSegment;
                while (distTravelledOnEdge - tol < edgeSize)
                {
                    distToSegment = GetWallDistance(ex, ey, segment, tol, n2.x, n2.y);
                    if (distToSegment - tol < 0.05)
                        return true;
                    ex += cosTheta * distToSegment;
                    ey += sinTheta * distToSegment;
                    distTravelledOnEdge += distToSegment;
                }

            }
            return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y){
            // Set wall vars
            double X1 = mapSegmentCorners[segment, 0, 0];
            double Y1 = mapSegmentCorners[segment, 0, 1];
            double X2 = mapSegmentCorners[segment, 1, 0];
            double Y2 = mapSegmentCorners[segment, 1, 1];
            double dist = 9999;

            // Put code here to calculated dist.
            // Calculate slope and intercept
            double angleSegmentPerpendicular = Math.PI / 2 + Math.Atan((Y2 - Y1) / (0.000001 + X2 - X1));
            double m = Math.Tan(angleSegmentPerpendicular);
            double b = y - m * x;

            // Get line intersection
            double x_intersect = (b - intercepts[segment]) / (slopes[segment] - m);
            double y_intersect = m * x_intersect + b;

            // Check for horiz/vert slopes
            if (Math.Abs(Y2 - Y1) < 0.001)
                y_intersect = Y1;
            if (Math.Abs(X2 - X1) < 0.001)
                x_intersect = X1;


            // Check to see if intersection LIES within segment
            double dist_intersect_corner1 = Math.Sqrt(Math.Pow(x_intersect - X1, 2) + Math.Pow(y_intersect - Y1, 2));
            double dist_intersect_corner2 = Math.Sqrt(Math.Pow(x_intersect - X2, 2) + Math.Pow(y_intersect - Y2, 2));
            if (dist_intersect_corner1 <= (segmentSizes[segment] + tol) && dist_intersect_corner2 <= (segmentSizes[segment] + tol))
            {
                dist = Math.Sqrt(Math.Pow(x - x_intersect, 2) + Math.Pow(y - y_intersect, 2));
            }

            // Check for distance to corners (for case where no intersection with segment
            double dist_point_corner1 = Math.Sqrt(Math.Pow(x - X1, 2) + Math.Pow(y - Y1, 2));
            double dist_point_corner2 = Math.Sqrt(Math.Pow(x - X2, 2) + Math.Pow(y - Y2, 2));
            dist = Math.Min(dist, dist_point_corner1);
            dist = Math.Min(dist, dist_point_corner2);

            return dist;
        }






    }
}
