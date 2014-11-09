/*	
	The MIT License (MIT)

	Copyright (c) 2014 Chandan Pawaskar (chandan.pawaskar@gmail.com)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/


//
// Do deformation transfer from source meshes to target meshes.
//
// Flags:
//
// -src/source <string>: Comma separated names of the source meshes. 
//		Deformation from first to (2nd, 3rd, ...) will be transfered to target meshes.
//		
// -tgt/target <string>: Comma separated names of the target meshes. 
//		Deformation be transfered to target mesh t0 and applied to 
//		remaining target meshes (2nd, 3rd, ...).
//		
// -log: Turns on logging. All intermediate computation results will be logged. 
//		This is very slow but is useful for debugging.
//		
// Examples:
//    // Deformation from mesh s0 to s1 is transfered to t0 and applied to t1
//  	// Similarly deformation to s2 is applied to t2 and so on...
//		defTransfer -src "s0,s1,s2,s3,s4,s5" -tgt "t0,t1,t2,t3,t4,t5";
// 

#include <maya/MGlobal.h>
#include <maya/MString.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MFloatMatrix.h>
#include <maya/MTime.h>
#include <maya/MArgList.h>
#include <maya/MPlug.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnCamera.h>
#include <maya/MDagPath.h>
#include <maya/MItDag.h>
#include <maya/MSelectionList.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MAnimControl.h>
#include <maya/MPxCommand.h>
#include <maya/MComputation.h>
#include <maya/MFnPlugin.h>

#include <maya/MFnMesh.h>
#include <maya/MFloatPointArray.h>

#include <iostream>
#include <vector>
#include <windows.h>

#include <DeformationTransferManager.h>

#define mCommandName "defTransfer"	// Command name

#define VERSION_ID   0							// Version number

#define ERR_HEADER "Command error 'defTransfer':\n"


class defTransferCmd : public MPxCommand
{
public:
		defTransferCmd();
    virtual		~defTransferCmd();

    virtual MStatus doIt ( const MArgList& args );

    static void* creator();

private:
	static MStatus nodeFromName(MString name, MObject & obj);
	MStatus parseArgs ( const MArgList& args );
	bool AreMeshesTriangulated(const std::vector<MDagPath> &dagPaths);
	void DisplayHelp();
	std::vector<MDagPath>	srcDagPaths;
	std::vector<MDagPath>	tgtDagPaths;

	bool bLog, bHelp;
	float regularization;
	DWORD tcStart;

	ConstrainedVertIDs constrainedVerts;
};

defTransferCmd::defTransferCmd() {bLog = false;}

defTransferCmd::~defTransferCmd() {}

MStatus defTransferCmd::nodeFromName(MString name, MObject & obj)
{
	MSelectionList tempList;
    tempList.add( name );
    if ( tempList.length() > 0 ) 
	{
		tempList.getDependNode( 0, obj );
		return MS::kSuccess;
	}
	return MS::kFailure;
}



//
// Read the arguments, and make sure they are consistant
///////////////////////////////////////////////////////////////////////////////
#define MATCH(str, shortName, longName) \
		(((str)==(shortName))||((str)==(longName)))
static int stringArg(const MArgList& args, unsigned int &indx, MString & res)
{
	if (indx < args.length())
	{
		MStatus stat;
		indx++;
		res = args.asString( indx, &stat );
		if (stat == MS::kSuccess) 
			return 1;
	}
	return 0;
}

static int doubleArg(const MArgList& args, unsigned int &indx, double & res)
{
	if (indx < args.length())
	{
		MStatus stat;
		indx++;
		res = args.asDouble( indx, &stat );
		if (stat == MS::kSuccess) 
			return 1;
	}
	return 0;
}

MStatus defTransferCmd::parseArgs( const MArgList& args )
{
	// Parse the arguments.
	MString arg;
	MStatus stat = MS::kSuccess;
	MString str;
	MObject cameraNode;
	MObject node;

	unsigned int i;

	constrainedVerts.clear();
	regularization = 0;

	for ( i = 0; i < args.length(); i++ ) {
		arg = args.asString( i, &stat );
		if (stat != MS::kSuccess)
			continue;

		if (MATCH(arg, "-h", "-help"))
		{
			bHelp = true;
			return MS::kSuccess;
		}
		if (MATCH(arg, "-src", "-source") && stringArg(args, i, str))
		{
			MStringArray strArr;
			str.split(',', strArr);
			int i,imax =strArr.length();

			for (i=0; i<imax; i++)
			{
				MString strName = strArr[i];
				if (nodeFromName(strName, node) == MS::kSuccess)
				{
					srcDagPaths.resize(srcDagPaths.size()+1);
					MDagPath::getAPathTo(node, srcDagPaths[srcDagPaths.size()-1]);
				}
				else
				{
					cerr << "Source mesh not found: " << strName << "\n";
					return MS::kFailure;
				}
			}
		}
		else if (MATCH(arg, "-tgt", "-target") && stringArg(args, i, str))
		{
			MStringArray strArr;
			str.split(',', strArr);
			int i,imax =strArr.length();

			for (i=0; i<imax; i++)
			{
				MString strName = strArr[i];
				if (nodeFromName(strName, node) == MS::kSuccess)
				{
					tgtDagPaths.resize(tgtDagPaths.size()+1);
					MDagPath::getAPathTo(node, tgtDagPaths[tgtDagPaths.size()-1]);
				}
			}
		}
		else if (MATCH(arg, "-cv", "-constrains") && stringArg(args, i, str))
		{
			MStringArray strArr;
			str.split(',', strArr);
			int i,imax =strArr.length();

			for (i=0; i<imax; i++)
			{
				constrainedVerts.push_back(atoi(strArr[i].asChar()));
			}
		}
		else if (arg == "-log")
		{
			bLog =true;
		}
		else if (MATCH(arg, "-reg", "-regularization") && stringArg(args, i, str))
		{
			regularization = str.asFloat();
		}
		else
		{
			fprintf(stderr, "Unknown argument '%s'\n", arg.asChar());
			fflush(stderr);
		}
	}

	return MS::kSuccess;
}

bool defTransferCmd::AreMeshesTriangulated(const std::vector<MDagPath> &dagPaths)
{
	MStatus status;
	int i,imax;
	imax = dagPaths.size();

	for (i=0; i<imax; i++)
	{
		MFnMesh fnMsh( dagPaths[i], &status ); 
		MIntArray nTrisSA, triVertsSA;

		if (status != MStatus::kSuccess)
		{
			cerr << dagPaths[i].partialPathName() << " is not a mesh\n";
			return false;
		}

		if (fnMsh.getTriangles(nTrisSA, triVertsSA))
		{
			int j,jmax = nTrisSA.length();
			for (j=0; j<jmax; j++)
			{
				if(nTrisSA[j] != 1)
				{
					cerr << "Mesh is not triangular: " << dagPaths[i].partialPathName() << "\n";
					return false;
				}
			}
		}
	}

	return true;
}

void defTransferCmd::DisplayHelp()
{
		MString help( "Do deformation transfer from source meshes to target meshes\n" );

		help += "//\t -src/source Comma separated names of the source meshes.\n";
		help += "//\t		Deformation from first to (2nd, 3rd, ...) will \n";
		help += "//\t		be transfered to target meshes.\n";
		help += "//\t -tgt/target Comma separated names of the target meshes.\n";
		help += "//\t		Deformation will be transfered using first target mesh \n";
		help += "//\t		and applied to remaining target meshes (2nd, 3rd, ...).\n";
		help += "//\t -cv/constrains Comma separated id's of vertices to be constrained.\n";
		help += "//\t		This is optional. If not specified, first vertex is constrained.\n";
		help += "//\t -log Turns on logging. All intermediate computation results will be logged.\n";
		help += "//\t		This can be very slow but is useful for debugging. \n";
		
		help += "//\t -h/-help	Displays this help.\n";

		displayInfo( help );

		cerr << help;
}

//
// Main routine
///////////////////////////////////////////////////////////////////////////////
MStatus defTransferCmd::doIt( const MArgList& args )
{
	cerr << "\nCommand 'defTransfer':\n";
	tcStart = GetTickCount();

	//Reset
	srcDagPaths.clear(); tgtDagPaths.clear(); bHelp = false;
	
	// parse the command arguments
	MStatus status = parseArgs(args);
	if (bHelp)
	{
		DisplayHelp();
		return MS::kSuccess;
	}

	if (status != MS::kSuccess) 
	{
		cerr << "Error parsing arguments.\n\n";
		return status;
	}


	// Do deformation transfer
	if (srcDagPaths.size() < 2 || tgtDagPaths.size() < 2)
	{
		cerr << "Minimum 2 source meshes and 2 target meshes must be specified.\n\n";
		return MS::kFailure;
	}

	if (!AreMeshesTriangulated(srcDagPaths) || !AreMeshesTriangulated(tgtDagPaths))
	{
		cerr << "Meshes must be triangulated.\n\n";
		return MS::kFailure;
	}

	// Get handles to the neutral source and target meshes
	MFnMesh fnMshSrcA( srcDagPaths[0], &status ); 
	MFnMesh fnMshTgtA( tgtDagPaths[0], &status ); 

	// Get pointers to the mesh vertex points arrays.
	const float* rawPtsSA = fnMshSrcA.getRawPoints(&status);
	const float* rawPtsTA = fnMshTgtA.getRawPoints(&status);

	int nTris = 	fnMshSrcA.numPolygons();
	int nVerts= 	fnMshSrcA.numVertices();

	MIntArray nTrisSA, triVertsSA;
	fnMshSrcA.getTriangles(nTrisSA, triVertsSA);

	DeformationTransferManager DTM; float *defLocs;
	DTM.SetLogging(bLog);
	DTM.SetRegularization(regularization);
	DTM.SetSourceATargetA(nTris, nVerts, &triVertsSA[0], rawPtsSA, rawPtsTA, &constrainedVerts);

	int j, jmax = min(srcDagPaths.size(), tgtDagPaths.size());
	for (j=1; j<jmax; j++)
	{
		MFnMesh fnMshSrcB( srcDagPaths[j], &status ); 
		MFnMesh fnMshTgtB( tgtDagPaths[j], &status ); 

		const float* rawPtsSB = fnMshSrcB.getRawPoints(&status);
		const float* rawPtsTB = fnMshTgtB.getRawPoints(&status);
		if (false == DTM.TransferDeformation(rawPtsSB, rawPtsTB, &defLocs))
		{
			cerr << "DeformationTransfer failed:\n" << DTM.GetLastError() << "\n\n";
			return MS::kFailure;
		}

		// Update target mesh points
		MPoint mpt;
		for (int i=0; i<nVerts; i++)
		{
			mpt.x = *(defLocs++);
			mpt.y = *(defLocs++);
			mpt.z = *(defLocs++);
			fnMshTgtB.setPoint(i, mpt);
		}
	}

	DWORD tcTotal = GetTickCount() - tcStart; 
	char str[1024];
	sprintf_s(str, "[nTris, nVerts] = [%u, %u], nTransfers = %u.\n", nTris, nVerts, jmax-1);
	sprintf_s(str, "%sCommand 'defTransfer': Succeeded. Time taken: %u ms.\n", str, tcTotal);
	cerr << str; displayInfo( str );
	
	return MS::kSuccess;
}

//
//
///////////////////////////////////////////////////////////////////////////////
void * defTransferCmd::creator() { return new defTransferCmd(); }

MStatus initializePlugin( MObject obj )
{
    MFnPlugin plugin( obj, "Chandan Pawaskar", "1.0", "Any");

    MStatus status = plugin.registerCommand(mCommandName,
											defTransferCmd::creator );
	if (!status) status.perror("registerCommand");

    return status;
}

MStatus uninitializePlugin( MObject obj )
{
    MFnPlugin plugin( obj );

    MStatus status = plugin.deregisterCommand(mCommandName);
	if (!status) status.perror("deregisterCommand");

    return status;
}
