/*
rigpp.cpp
Definition of the main Maya node that compiles C++ to LLVM IR and execute it.
*/
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>

#include <maya/MPlugArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MPointArray.h>
#include <maya/MMatrixArray.h>
#include <maya/MStringArray.h>
#include <maya/MAngle.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnPluginData.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnNurbsCurveData.h>
#include <maya/MFnStringData.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MFnDependencyNode.h>

#include <maya/MFileIO.h>
#include <maya/MFileObject.h>

#include "rigpp.h"
#include "rigpp_types.h"
#include "utils.hpp"

#if _MSC_VER >= 1932 // Visual Studio 2022 version 17.2+
#pragma comment(linker, "/alternatename:__imp___std_init_once_complete=__imp_InitOnceComplete")
#pragma comment(linker, "/alternatename:__imp___std_init_once_begin_initialize=__imp_InitOnceBeginInitialize")
#endif

#define ATTR_PREFIX string("attr_")

string JITHelper::ROOT_PATH;

MTypeId Rigpp::typeId(1274443);
MObject Rigpp::attr_source;
MObject Rigpp::attr_debugMode;

inline bool isPlugOutput(const MPlug& plug)
{
	const string plugLongName(
		plug.partialName(
			false, // includeNodeName
			false, // includeNonMandatoryIndices
			false, // includeInstancedIndices
			false, // useAlias
			false, // useFullAttributePath
			true).asChar()); // useLongNames

	return startswith(plugLongName, "out_") ||
		   (plugLongName.size() > 3 && startswith(plugLongName, "out") && plugLongName[3] >= 65 && plugLongName[3] <= 90) || // like outUppercase (U in A..Z)
		   startswith(plugLongName, "output");
}

// Exported functions must be in C style
void c_print(const char* s) { MGlobal::displayInfo(s); }
void c_error(const char* s) { MGlobal::displayError(s); }

template<typename T>
inline void convertToMayaType(T a, T& out) { out = a; }
void convertToMayaType(const Vector& v, MVector& out) { out = MVector(v.x, v.y, v.z); }
void convertToMayaType(const Vector& v, MPoint& out) { out = MPoint(v.x, v.y, v.z); }
void convertToMayaType(const Matrix& m, MMatrix& out)
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			out[i][j] = m.x[i][j];
}

void convertToMayaType(const NurbsCurve& crv, MObject& out)
{
	if (crv.cvs.empty())
		return;

	MPointArray cvs(crv.cvs.size()); // curve CVs	
	for (int i = 0; i < crv.cvs.size(); i++)
		convertToMayaType(crv.cvs[i], cvs[i]);

	MDoubleArray knots(crv.knots.size());
	for (size_t i = 1; i < crv.knots.size() - 1; i++) // skip [0] and [-1]
		knots[i] = crv.knots[i];

	MFnNurbsCurveData curveDataFn;
	out = curveDataFn.create();

	MStatus stat;
	MFnNurbsCurve curveFn;
	curveFn.create(cvs, knots, crv.degree, MFnNurbsCurve::kOpen, false, false, out, &stat);
}

void convertToMayaType(uint8_t v, MDataHandle handle) { handle.setBool((bool)v); }
void convertToMayaType(short v, MDataHandle handle) { handle.setShort(v); }
void convertToMayaType(int v, MDataHandle handle) { handle.setInt(v); }
void convertToMayaType(float v, MDataHandle handle) { handle.setFloat(v); }
void convertToMayaType(double v, MDataHandle handle) { handle.setDouble(v); }
void convertToMayaType(const Vector &v, MDataHandle handle) { handle.setMVector(MVector(v.x, v.y, v.z)); }
void convertToMayaType(const Matrix& m, MDataHandle handle) 
{ 
	MMatrix out;
	convertToMayaType(m, out);
	handle.setMMatrix(out); 
}

void convertToMayaType(const string& s, MDataHandle handle)
{
	MFnStringData stringFn;
	MObject out = stringFn.create(s.c_str());
	handle.setMObject(out);
}

void convertToMayaType(const NurbsCurve& crv, MDataHandle handle)
{
	MObject out;
	convertToMayaType(crv, out);
	handle.setMObject(out);
}

template<typename T>
inline void convertFromMayaType(T a, T& out) { out = a; }
void convertFromMayaType(const MVector& v, Vector& out) { out = Vector(v.x, v.y, v.z); }
void convertFromMayaType(const MMatrix& m, Matrix& out)
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			out.x[i][j] = m[i][j];
}

void convertFromMayaType(MObject curveObject, NurbsCurve& out)
{
	MFnNurbsCurve inputCurveFn(curveObject);

	out.degree = inputCurveFn.degree();

	MDoubleArray knots;
	inputCurveFn.getKnots(knots);

	const int N_KNOTS = knots.length();

	out.knots.resize(N_KNOTS + 2);

	for (int j = 0; j < N_KNOTS; j++)
		out.knots[j + 1] = knots[j];

	out.knots[0] = 2.0 * knots[1] - knots[2];
	out.knots[N_KNOTS + 1] = 2.0 * knots[N_KNOTS - 1] - knots[N_KNOTS - 2];

	MPointArray cvs;
	inputCurveFn.getCVs(cvs, MSpace::kWorld);
	out.cvs.resize(cvs.length());

	for (int j = 0; j < cvs.length(); j++)
		convertFromMayaType(cvs[j], out.cvs[j]);
}

void convertFromMayaType(const MString& s, string& out) { out = s.asChar(); }

void convertFromMayaType(const MDataHandle handle, uint8_t& out) { out = (uint8_t)handle.asBool(); } // we cannot take & of vector<bool>
void convertFromMayaType(const MDataHandle handle, short& out) { out = handle.asShort(); }
void convertFromMayaType(const MDataHandle handle, int& out) { out = handle.asInt(); }
void convertFromMayaType(const MDataHandle handle, float& out) { out = handle.asFloat(); }
void convertFromMayaType(const MDataHandle handle, double& out) { out = handle.asDouble(); }
void convertFromMayaType(const MDataHandle handle, Vector& out) { convertFromMayaType(handle.asVector(), out); }
void convertFromMayaType(const MDataHandle handle, Matrix& out) { convertFromMayaType(handle.asMatrix(), out); }
void convertFromMayaType(const MDataHandle handle, NurbsCurve& out) { convertFromMayaType(handle.asNurbsCurve(), out); }
void convertFromMayaType(const MDataHandle handle, string& out) { convertFromMayaType(handle.asString(), out); }

template<typename T>
void getInputArray(MDataBlock dataBlock, MObject attribute, vector<T> &data)
{
	MArrayDataHandle handle = dataBlock.inputArrayValue(attribute);
	data.resize(handle.elementCount());
	for (int i = 0; i < handle.elementCount(); i++)
	{
		handle.jumpToArrayElement(i);
		convertFromMayaType(handle.inputValue(), data[i]);
	}
}

void Rigpp::getInputs(MDataBlock& dataBlock)
{
	for (auto& plugMemInfo : attributes)
	{
		if (plugMemInfo.second.plug.isNull() || isPlugOutput(plugMemInfo.second.plug))
			continue;

		MObject attr = plugMemInfo.second.plug.attribute();
		MFnAttribute attrFn(attr);
		const string attrName(attrFn.name().asChar());

		if (attr.apiType() == MFn::kNumericAttribute || attr.apiType() == MFn::kAttribute3Double)
		{
			MFnNumericAttribute nAttr(attr);

			if (nAttr.unitType() == MFnNumericData::kBoolean)
			{
				if (nAttr.isArray())
					getInputArray(dataBlock, attr, any_cast<vector<uint8_t>&>(plugMemInfo.second.value));
				else
					convertFromMayaType(dataBlock.inputValue(attr), any_cast<uint8_t&>(plugMemInfo.second.value));
			}

			else if (nAttr.unitType() == MFnNumericData::kInt)
			{
				if (nAttr.isArray())
					getInputArray(dataBlock, attr, any_cast<vector<int>&>(plugMemInfo.second.value));
				else
					convertFromMayaType(dataBlock.inputValue(attr), any_cast<int&>(plugMemInfo.second.value));
			}

			else if (nAttr.unitType() == MFnNumericData::kFloat)
			{
				if (nAttr.isArray())
					getInputArray(dataBlock, attr, any_cast<vector<float>&>(plugMemInfo.second.value));
				else
					convertFromMayaType(dataBlock.inputValue(attr), any_cast<float&>(plugMemInfo.second.value));
			}

			else if (nAttr.unitType() == MFnNumericData::kDouble)
			{
				if (nAttr.isArray())
					getInputArray(dataBlock, attr, any_cast<vector<double>&>(plugMemInfo.second.value));
				else
					convertFromMayaType(dataBlock.inputValue(attr), any_cast<double&>(plugMemInfo.second.value));
			}

			else if (nAttr.unitType() == MFnNumericData::k3Double)
			{
				if (nAttr.isArray())
					getInputArray(dataBlock, attr, any_cast<vector<Vector>&>(plugMemInfo.second.value));
				else
					convertFromMayaType(dataBlock.inputValue(attr), any_cast<Vector&>(plugMemInfo.second.value));
			}
		}

		else if (attr.apiType() == MFn::kTypedAttribute)
		{
			MFnTypedAttribute tAttr(attr);
			if (tAttr.attrType() == MFnData::kString)
			{
				if (tAttr.isArray())
					getInputArray(dataBlock, attr, any_cast<vector<string>&>(plugMemInfo.second.value));
				else
					convertFromMayaType(dataBlock.inputValue(attr), any_cast<string&>(plugMemInfo.second.value));
			}

			if (tAttr.attrType() == MFnData::kNurbsCurve)
			{
				if (tAttr.isArray())
					getInputArray(dataBlock, attr, any_cast<vector<NurbsCurve>&>(plugMemInfo.second.value));
				else
					convertFromMayaType(dataBlock.inputValue(attr), any_cast<NurbsCurve&>(plugMemInfo.second.value));
			}
		}

		else if (attr.apiType() == MFn::kMatrixAttribute)
		{
			if (attrFn.isArray())
				getInputArray(dataBlock, attr, any_cast<vector<Matrix>&>(plugMemInfo.second.value));
			else
				convertFromMayaType(dataBlock.inputValue(attr), any_cast<Matrix&>(plugMemInfo.second.value));
		}

		else if (attr.apiType() == MFn::kEnumAttribute)
		{
			if (attrFn.isArray())
				getInputArray(dataBlock, attr, any_cast<vector<short>&>(plugMemInfo.second.value));
			else
				convertFromMayaType(dataBlock.inputValue(attr), any_cast<short&>(plugMemInfo.second.value));
		}
	} // for	
}

template<typename T>
void setOutputArray(MDataBlock& dataBlock, MObject attribute, const vector<T>& data)
{
	MArrayDataHandle handle = dataBlock.outputArrayValue(attribute);
	MArrayDataBuilder builder(&dataBlock, attribute, data.size());

	for (int i = 0; i < data.size(); i++)
	{
		MDataHandle h = builder.addElement(i);
		convertToMayaType(data[i], h);
	}
	handle.set(builder);
	handle.setAllClean();
}

void Rigpp::setOutputs(MDataBlock& dataBlock)
{
	for (auto& plugMemInfo : attributes)
	{
		if (plugMemInfo.second.plug.isNull() || !isPlugOutput(plugMemInfo.second.plug))
			continue;

		MObject attr = plugMemInfo.second.plug.attribute();
		MFnAttribute attrFn(attr);
		const string attrName(attrFn.name().asChar());

		if (attr.apiType() == MFn::kNumericAttribute || attr.apiType() == MFn::kAttribute3Double)
		{
			MFnNumericAttribute nAttr(attr);

			if (nAttr.unitType() == MFnNumericData::kBoolean)
			{
				if (nAttr.isArray())
					setOutputArray(dataBlock, attr, any_cast<vector<uint8_t>&>(plugMemInfo.second.value));
				else
					convertToMayaType(any_cast<uint8_t&>(plugMemInfo.second.value), dataBlock.outputValue(attr));
			}

			else if (nAttr.unitType() == MFnNumericData::kInt)
			{
				if (nAttr.isArray())
					setOutputArray(dataBlock, attr, any_cast<vector<int>&>(plugMemInfo.second.value));
				else
					convertToMayaType(any_cast<int&>(plugMemInfo.second.value), dataBlock.outputValue(attr));
			}

			else if (nAttr.unitType() == MFnNumericData::kFloat)
			{
				if (nAttr.isArray())
					setOutputArray(dataBlock, attr, any_cast<vector<float>&>(plugMemInfo.second.value));
				else
					convertToMayaType(any_cast<float&>(plugMemInfo.second.value), dataBlock.outputValue(attr));
			}

			else if (nAttr.unitType() == MFnNumericData::kDouble)
			{
				if (nAttr.isArray())
					setOutputArray(dataBlock, attr, any_cast<vector<double>&>(plugMemInfo.second.value));
				else
					convertToMayaType(any_cast<double&>(plugMemInfo.second.value), dataBlock.outputValue(attr));
			}

			else if (nAttr.unitType() == MFnNumericData::k3Double)
			{
				if (nAttr.isArray())
					setOutputArray(dataBlock, attr, any_cast<vector<Vector>&>(plugMemInfo.second.value));
				else
					convertToMayaType(any_cast<Vector&>(plugMemInfo.second.value), dataBlock.outputValue(attr));
			}
		}

		else if (attr.apiType() == MFn::kTypedAttribute)
		{
			MFnTypedAttribute tAttr(attr);
			if (tAttr.attrType() == MFnData::kString)
			{
				if (tAttr.isArray())
					setOutputArray(dataBlock, attr, any_cast<vector<string>&>(plugMemInfo.second.value));
				else
					convertToMayaType(any_cast<string&>(plugMemInfo.second.value), dataBlock.outputValue(attr));
			}

			if (tAttr.attrType() == MFnData::kNurbsCurve)
			{
				if (tAttr.isArray())
					setOutputArray(dataBlock, attr, any_cast<vector<NurbsCurve>&>(plugMemInfo.second.value));
				else
					convertToMayaType(any_cast<NurbsCurve&>(plugMemInfo.second.value), dataBlock.outputValue(attr));
			}
		}

		else if (attr.apiType() == MFn::kMatrixAttribute)
		{
			if (attrFn.isArray())
				setOutputArray(dataBlock, attr, any_cast<vector<Matrix>&>(plugMemInfo.second.value));
			else
				convertToMayaType(any_cast<Matrix&>(plugMemInfo.second.value), dataBlock.outputValue(attr));
		}

		else if (attr.apiType() == MFn::kEnumAttribute)
		{
			if (attrFn.isArray())
				setOutputArray(dataBlock, attr, any_cast<vector<short>&>(plugMemInfo.second.value));
			else
				convertToMayaType(any_cast<short&>(plugMemInfo.second.value), dataBlock.outputValue(attr));
		}

		dataBlock.setClean(attr);
	}
}

void measureTime(const string& name, const chrono::steady_clock::time_point& start, const chrono::steady_clock::time_point& end)
{
	const auto count = chrono::duration_cast<chrono::microseconds>(end - start).count();
	const string msg = formatString("%s: %.2fms", name.c_str(), count / 1000.0);
	MGlobal::displayInfo(msg.c_str());
};

MStatus Rigpp::compute(const MPlug& plug, MDataBlock& dataBlock)
{
	if (!isPlugOutput(plug))
		return MS::kInvalidParameter;

	if (!mainFunction)
	{
		MGlobal::displayError(thisNodeFn.name() + ": invalid main function");
		return MS::kFailure;
	}

	const DebugMode debugMode = (DebugMode)dataBlock.inputValue(attr_debugMode).asShort();

	chrono::steady_clock::time_point startTime, getInputsTime, jitTime, setOutputsTime;
	startTime = chrono::steady_clock::now();

	getInputs(dataBlock);

	getInputsTime = chrono::steady_clock::now();
	mainFunction();
	jitTime = chrono::steady_clock::now();
	
	setOutputs(dataBlock);
	dataBlock.setClean(plug);

	setOutputsTime = chrono::steady_clock::now();

	if (debugMode == DebugMode::DebugPerfomance)
	{
		MGlobal::displayInfo(thisNodeFn.name() + ": perfomance");
		measureTime("Get inputs", startTime, getInputsTime);
		measureTime("C++ jit", getInputsTime, jitTime);
		measureTime("Set outputs", jitTime, setOutputsTime);
		measureTime("Overall", startTime, setOutputsTime);
	}

	return MS::kSuccess;
}

#define SET_SYMBOL(suffix, type){\
	auto& item = attributes[attrName]; \
	item.plug = attrPlug; \
	item.value = make_any<type>(); \
	symbols["?" + ATTR_PREFIX + attrName + "@@" + suffix] = &any_cast<type&>(item.value);}

void Rigpp::updateSymbols()
{
	symbols.clear();
	attributes.clear();

	// export some functions
	symbols["c_print"] = &c_print;
	symbols["c_error"] = &c_error;	

	for (auto i = 0; i < thisNodeFn.attributeCount(); i++)
	{
		MObject attr = thisNodeFn.attribute(i);
		MPlug attrPlug = thisNodeFn.findPlug(attr, false);
		MFnAttribute attrFn(attr);
		const string attrName(attrFn.name().asChar());

		if (!attrPlug.isDynamic() || attrPlug.isChild())
			continue;

		if (attr.apiType() == MFn::kNumericAttribute || attr.apiType() == MFn::kAttribute3Double)
		{
			MFnNumericAttribute nAttr(attr);

			if (nAttr.unitType() == MFnNumericData::kBoolean)
			{
				if (nAttr.isArray())
					SET_SYMBOL("3V?$vector@EV?$allocator@E@std@@@std@@A", vector<uint8_t>)
				else
					SET_SYMBOL("3EA", uint8_t)
			}

			else if (nAttr.unitType() == MFnNumericData::kInt)
			{
				if (nAttr.isArray())
					SET_SYMBOL("3V?$vector@HV?$allocator@H@std@@@std@@A", vector<int>)
				else
					SET_SYMBOL("3HA", int)
			}

			else if (nAttr.unitType() == MFnNumericData::kFloat)
			{
				if (nAttr.isArray())
					SET_SYMBOL("3V?$vector@MV?$allocator@M@std@@@std@@A", vector<float>)
				else
					SET_SYMBOL("3MA", float)
			}

			else if (nAttr.unitType() == MFnNumericData::kDouble)
			{
				if (nAttr.isArray())
					SET_SYMBOL("3V?$vector@NV?$allocator@N@std@@@std@@A", vector<double>)
				else
					SET_SYMBOL("3NA", double)
			}

			else if (nAttr.unitType() == MFnNumericData::k3Double)
			{
				if (nAttr.isArray())
					SET_SYMBOL("3V?$vector@VVector@@V?$allocator@VVector@@@std@@@std@@A", vector<Vector>)
				else
					SET_SYMBOL("3VVector@@A", Vector)
			}
		}

		else if (attr.apiType() == MFn::kTypedAttribute)
		{
			MFnTypedAttribute tAttr(attr);
			if (tAttr.attrType() == MFnData::kString)
			{
				if (tAttr.isArray())
					SET_SYMBOL("3V?$vector@V?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@V?$allocator@V?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@@2@@std@@A", vector<string>)
				else
					SET_SYMBOL("3V?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@A", string)
			}

			if (tAttr.attrType() == MFnData::kNurbsCurve)
			{
				if (tAttr.isArray())
					;
				else
					SET_SYMBOL("3VNurbsCurve@@A", NurbsCurve)
			}
		}

		else if (attr.apiType() == MFn::kMatrixAttribute)
		{
			if (attrFn.isArray())
				SET_SYMBOL("3V?$vector@VMatrix@@V?$allocator@VMatrix@@@std@@@std@@A", vector<Matrix>)
			else
				SET_SYMBOL("3VMatrix@@A", Matrix)
		}

		else if (attr.apiType() == MFn::kEnumAttribute)
		{
			if (attrFn.isArray())
				SET_SYMBOL("3V?$vector@FV?$allocator@F@std@@@std@@A", vector<short>)
			else
				SET_SYMBOL("3FA", short)
		}
	} // for	
}

SourceType getSourceType(const string& source)
{
	SourceType sourceType;

	if (endswith(source, ".cpp"))
		sourceType = SourceType::CppFile;

	else if (endswith(source, ".bc") || endswith(source, ".ll"))
		sourceType = SourceType::LLVM_IRFile;

	else
		sourceType = SourceType::Cpp;

	return sourceType;
}

void readFromFile(const string& path, string& content)
{
	ifstream myfile(path);
	string line;
	while (getline(myfile, line))
		content += line;
	myfile.close();
}

bool Rigpp::setupJit(const string &source)
{
	mainFunction = nullptr;

	jitHelper.initJit();
	
	SourceType sourceType = getSourceType(source);

	if (sourceType == SourceType::Cpp)
	{
		string log;
		if (!jitHelper.compileCpp(replaceString(source, "@", ATTR_PREFIX), log))
		{
			MGlobal::displayError(thisNodeFn.name() + ": " + MString(log.c_str()));			
			return false;
		}
	}

	else if (sourceType == SourceType::CppFile)
	{
		MFileObject fileObject;
		fileObject.setRawFullName(source.c_str());
		const string filePath = fileObject.resolvedFullName().asChar();

		string cppcode;
		readFromFile(filePath, cppcode);

		string log;
		const string outputIRPath = replaceString(filePath, ".cpp", ".bc");
		if (!jitHelper.compileCpp(replaceString(cppcode, "@", ATTR_PREFIX), log, outputIRPath))
		{
			MGlobal::displayError(thisNodeFn.name() + ": " + MString(log.c_str()));
			return false;
		}
	}

	else if (sourceType == SourceType::LLVM_IRFile)
	{
		MFileObject fileObject;
		fileObject.setRawFullName(source.c_str());
		const string filePath = fileObject.resolvedFullName().asChar();

		string log;
		if (!jitHelper.compileIR(filePath, log))
		{
			MGlobal::displayError(thisNodeFn.name() + ": " + MString(log.c_str()));
			return false;
		}
	}

	updateSymbols();
	jitHelper.addSymbols(symbols);
	
	mainFunction = jitHelper.lookup<MainFunctionType>("main");
	if (!mainFunction)
	{
		MGlobal::displayError(thisNodeFn.name() + ": lookup failed");
		return false;
	}

	return true;
}

MStatus Rigpp::setDependentsDirty(const MPlug& plugBeingDirtied, MPlugArray& affectedPlugs)
{
	if (isPlugOutput(plugBeingDirtied) || 
		plugBeingDirtied == attr_source || 
		plugBeingDirtied == attr_debugMode)
		return MS::kSuccess;

	for (int i = 0; i < thisNodeFn.attributeCount(); i++)
	{
		MObject attr = thisNodeFn.attribute(i);
		MPlug plug = thisNodeFn.findPlug(attr, false);

		if (isPlugOutput(plug))
		{
			if (plug.isArray())
			{
				affectedPlugs.append(plug);
				for (int k = 0; k < plug.numElements(); k++) // add each element
					affectedPlugs.append(plug.elementByPhysicalIndex(k));
			}
			else
				affectedPlugs.append(plug);
		}
	} // for i

	return MPxNode::setDependentsDirty(plugBeingDirtied, affectedPlugs);
}

bool Rigpp::setInternalValue(const MPlug& plug, const MDataHandle& dataHandle)
{
	if (plug == attr_source)
	{
		const string source = dataHandle.asString().asChar();		

		if (setupJit(source))
			MGlobal::displayInfo(thisNodeFn.name() + ": compiled successfully");
	}

	return MPxNode::setInternalValue(plug, dataHandle);
}

void Rigpp::copyInternalData(MPxNode* fromNode) // for duplicates
{
	const string source(thisNodeFn.findPlug(attr_source, true).asString().asChar());

	if (setupJit(source))
		MGlobal::displayInfo(thisNodeFn.name() + ": compiled successfully");
}

void Rigpp::postConstructor()
{
	thisNodeFn.setObject(thisMObject());
	setExistWithoutInConnections(true);
	setExistWithoutOutConnections(true);
}

MStatus Rigpp::initialize()
{
	MFnNumericAttribute nAttr;
	MFnTypedAttribute tAttr;
	MFnEnumAttribute eAttr;

	attr_source = tAttr.create("_source", "_source", MFnData::kString, MFnStringData().create("#include <rigpp>\nint main(){}"));
	tAttr.setInternal(true);
	addAttribute(attr_source);

	attr_debugMode = eAttr.create("_debugMode", "_debugMode");
	eAttr.addField("(No debug)", 0);
	eAttr.addField("Perfomance", 1);
	addAttribute(attr_debugMode);

	return MS::kSuccess;
}