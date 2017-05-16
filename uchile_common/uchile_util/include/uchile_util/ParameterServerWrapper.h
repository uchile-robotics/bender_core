/*
 * ParameterServerWrapper.h
 *
 *      Author: matias.pavez
 */

#ifndef PARAMETERSERVERWRAPPER_H_
#define PARAMETERSERVERWRAPPER_H_

// C, C++
#include <cstdio> // for EOF
#include <string>
#include <sstream>
#include <vector>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

namespace uchile_util {

class ParameterServerWrapper {

private:
	ros::NodeHandle priv;

public:
	ParameterServerWrapper(std::string name = "~");
	virtual ~ParameterServerWrapper();

	bool getParameter(std::string key, int & parameter, int default_value);
	bool getParameter(std::string key, bool & parameter, bool default_value);
	bool getParameter(std::string key, float & parameter, float default_value);
	bool getParameter(std::string key, double & parameter, double default_value);
	bool getParameter(std::string key, std::string & parameter, std::string default_value);
	bool getParameter(std::string key, std::vector<bool> & parameter, std::vector<bool> default_value);
	bool getParameter(std::string key, std::vector<int> & parameter, std::vector<int> default_value);
	bool getParameter(std::string key, std::vector<float> & parameter, std::vector<float> default_value);
	bool getParameter(std::string key, std::vector<double> & parameter, std::vector<double> default_value);
	bool getParameter(std::string key, std::vector<std::string> & parameter, std::vector<std::string> default_value);
	bool getParameter(std::string key, std::map<std::string,bool> & parameter, std::map<std::string,bool> default_value);
	bool getParameter(std::string key, std::map<std::string,int> & parameter, std::map<std::string,int> default_value);
	bool getParameter(std::string key, std::map<std::string,float> & parameter, std::map<std::string,float> default_value);
	bool getParameter(std::string key, std::map<std::string,double> & parameter, std::map<std::string,double> default_value);
	bool getParameter(std::string key, std::map<std::string,std::string> & parameter, std::map<std::string,std::string> default_value);
	bool getParameter(std::string key, geometry_msgs::Polygon & parameter, geometry_msgs::Polygon default_value);

private:

	/** - - - - Methods for Polygon Handling - - - - **/
	std::vector<std::vector<float> > parseVVF( const std::string& input, std::string& error_return );
	float getNumberFromXMLRPC( XmlRpc::XmlRpcValue& value, const std::string& full_param_name );
	void readPolygonFromXMLRPC( XmlRpc::XmlRpcValue& polygon_xmlrpc, const std::string& full_param_name, std::vector<geometry_msgs::Point32> & points);
	bool readPolygonFromString( const std::string& polygon_string, std::vector<geometry_msgs::Point32> & points);
	void writePolygonToParam( const std::string& full_param_name, std::vector<geometry_msgs::Point32> points );
};


inline ParameterServerWrapper::ParameterServerWrapper(std::string name) {

	priv = ros::NodeHandle(name);
}

inline ParameterServerWrapper::~ParameterServerWrapper() {

}

inline bool ParameterServerWrapper::getParameter(std::string key, int & parameter, int default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom '"<< key <<": " << parameter << "'");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default '"<< key <<": " << parameter << "'");
		ret_val = false;
	}

	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, bool & parameter, bool default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom '"<< key <<": " << parameter << "'");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default '"<< key <<": " << parameter << "'");
		ret_val = false;
	}

	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, float & parameter, float default_value) {

	bool ret_val;
	double d_parameter;
	if ( priv.getParam(key, d_parameter) ) {

		parameter = (float)d_parameter;
		ROS_INFO_STREAM(" - using custom '"<< key <<": " << parameter << "'");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default '"<< key <<": " << parameter << "'");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, double & parameter, double default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom '"<< key <<": " << parameter << "'");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default '"<< key <<": " << parameter << "'");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::string & parameter, std::string default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom '"<< key <<": " << parameter << "'");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default '"<< key <<": " << parameter << "'");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::vector<bool> & parameter, std::vector<bool> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom boolean list for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default boolean list for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::vector<int> & parameter, std::vector<int> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom integer list for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default integer list for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
	/*
	XmlRpc::XmlRpcValue list;
	parameter.clear();

	if ( priv.getParam(key, list) ) {

		ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for (int32_t i = 0; i < list.size(); ++i) {

			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
			parameter.push_back(static_cast<int>(list[i]));
		}

		ROS_INFO_STREAM(" - using custom integer list for '"<< key << "' which has (" << list.size() << ") points");
		ret_val = true;

	}*/
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::vector<float> & parameter, std::vector<float> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom floating point list (float) for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default floating points list (float) for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::vector<double> & parameter, std::vector<double> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom floating point (double) list for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default floating points (double) list for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;

	/*
	XmlRpc::XmlRpcValue list;
	parameter.clear();

	if ( priv.getParam(key, list) ) {

		ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for (int32_t i = 0; i < list.size(); ++i) {

			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			parameter.push_back(static_cast<double>(list[i]));
		}

		ROS_INFO_STREAM(" - using custom floating point list for '"<< key << "' which has (" << list.size() << ") points");
		ret_val = true;

	}*/
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::vector<std::string> & parameter, std::vector<std::string> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom string list for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default string list for '"<< key << "' which has (" << parameter.size() << ") points");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::map<std::string,bool> & parameter, std::map<std::string,bool> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom boolean map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default boolean map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::map<std::string,int> & parameter, std::map<std::string,int> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom integer map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default integer map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::map<std::string,float> & parameter, std::map<std::string,float> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom float map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default float map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::map<std::string,double> & parameter, std::map<std::string,double> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom double map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default double map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::getParameter(std::string key, std::map<std::string,std::string> & parameter, std::map<std::string,std::string> default_value) {

	bool ret_val;
	if ( priv.getParam(key,parameter) ) {

		ROS_INFO_STREAM(" - using custom string map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = true;

	} else {

		parameter = default_value;
		ROS_WARN_STREAM(" - using default string map for '"<< key << "' which has (" << parameter.size() << ") items");
		ret_val = false;
	}
	priv.setParam(key,parameter);
	return ret_val;
}

inline bool ParameterServerWrapper::readPolygonFromString( const std::string& polygon_string, std::vector<geometry_msgs::Point32> & points) {

	std::string error;
	std::vector<std::vector<float> > vvf = parseVVF( polygon_string, error );
	if( error != "" ) {
		ROS_ERROR( "Error parsing polygon parameter: '%s'", error.c_str() );
		ROS_ERROR( " Footprint string was '%s'.", polygon_string.c_str() );
		return false;
	}

	// convert vvf into points.
	if( vvf.size() < 3 ) {
		ROS_ERROR( "You must specify at least three points for the polygon" );
		return false;
	}

	points.clear();
	points.reserve( vvf.size() );
	for( unsigned int i = 0; i < vvf.size(); i++ ) {

		if( vvf[ i ].size() == 2 ) {

			geometry_msgs::Point32 point;
			point.x = vvf[ i ][ 0 ];
			point.y = vvf[ i ][ 1 ];
			point.z = 0;
			points.push_back( point );

		} else {
			ROS_ERROR( "Points in the polygon specification must be pairs of numbers. Found a point with %d numbers.", int( vvf[ i ].size() ));
			return false;
		}
	}
	return true;
}

inline void ParameterServerWrapper::writePolygonToParam( const std::string& full_param_name, std::vector<geometry_msgs::Point32> points ) {

	std::ostringstream oss;
	bool first = true;
	for( unsigned int i = 0; i < points.size(); i++ ) {
		geometry_msgs::Point32& p = points[ i ];
		if( first ) {
			oss << "[[" << p.x << "," << p.y << "]";
			first = false;

		} else {
			oss << ",[" << p.x << "," << p.y << "]";
		}
	}
	oss << "]";
	priv.setParam( full_param_name, oss.str().c_str() );
}

inline float ParameterServerWrapper::getNumberFromXMLRPC( XmlRpc::XmlRpcValue& value, const std::string& full_param_name ) {

	// Make sure that the value we're looking at is either a double or an int.
	if( value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
		value.getType() != XmlRpc::XmlRpcValue::TypeDouble ) {

		std::string& value_string = value;
		ROS_FATAL( "Values in the polygon specification (param %s) must be numbers. Found value %s.", full_param_name.c_str(), value_string.c_str() );
		throw std::runtime_error("Values in the polygon specification must be numbers");
	}
	return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

inline void ParameterServerWrapper::readPolygonFromXMLRPC( XmlRpc::XmlRpcValue& polygon_xmlrpc, const std::string& full_param_name, std::vector<geometry_msgs::Point32> & points) {

	// Make sure we have an array of at least 3 elements.
	if( polygon_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || polygon_xmlrpc.size() < 3 )
	{
		ROS_FATAL( "The polygon must be specified as list of lists on the parameter server, %s was specified as %s",
				full_param_name.c_str(), std::string( polygon_xmlrpc ).c_str() );
		throw std::runtime_error( "The polygon must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
	}

	points.clear();
	geometry_msgs::Point32 pt;
	for( int i = 0; i < polygon_xmlrpc.size(); ++i )
	{
		// Make sure each element of the list is an array of size 2. (x and y coordinates)
		XmlRpc::XmlRpcValue point = polygon_xmlrpc[ i ];
		if( point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
				point.size() != 2 )
		{
			ROS_FATAL( "The footprint (parameter %s) must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.", full_param_name.c_str() );
			throw std::runtime_error( "The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form" );
		}
		pt.x = getNumberFromXMLRPC( point[ 0 ], full_param_name );
		pt.y = getNumberFromXMLRPC( point[ 1 ], full_param_name );
		pt.z = 0;
		points.push_back( pt );
	}
}

inline bool ParameterServerWrapper::getParameter(std::string key, geometry_msgs::Polygon & parameter, geometry_msgs::Polygon default_value) {

	bool get_ok = false;
	std::string full_param_name;
	if( priv.searchParam( key, full_param_name )) {

		XmlRpc::XmlRpcValue polygon_xmlrpc;
		priv.getParam( full_param_name, polygon_xmlrpc );

		if( polygon_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString ) {

			if( readPolygonFromString( std::string( polygon_xmlrpc ), parameter.points)) {
				writePolygonToParam( full_param_name, parameter.points );
				get_ok = true;
			}

		} else if( polygon_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray ) {

			readPolygonFromXMLRPC( polygon_xmlrpc, full_param_name, parameter.points);
			writePolygonToParam( full_param_name, parameter.points );
			get_ok = true;
		}
	}

	if (get_ok) {
		ROS_INFO_STREAM(" - using custom polygon for '"<< key << "' which has (" << parameter.points.size() << ") points");
		return true;
	} else {
		parameter = default_value;
		ROS_WARN_STREAM(" - using default polygon for '"<< key << "' which has (" << parameter.points.size() << ") points");
		return false;
	}
}

inline std::vector<std::vector<float> > ParameterServerWrapper::parseVVF( const std::string& input, std::string& error_return ) {

	std::vector<std::vector<float> > result;
	std::stringstream input_ss( input );
	int depth = 0;
	std::vector<float> current_vector;

	while( !!input_ss && !input_ss.eof() ) {

		switch( input_ss.peek() ) {

		case EOF:
			break;
		case '[':
			depth++;
			if( depth > 2 ) {
				error_return = "Array depth greater than 2";
				return result;
			}
			input_ss.get();
			current_vector.clear();
			break;
		case ']':
			depth--;
			if( depth < 0 ) {
				error_return = "More close ] than open [";
				return result;
			}
			input_ss.get();
			if( depth == 1 ) {
				result.push_back( current_vector );
			}
			break;
		case ',':
		case ' ':
		case '\t':
			input_ss.get();
			break;
		default: // All other characters should be part of the numbers.
			if( depth != 2 ) {
				std::stringstream err_ss;
				err_ss << "Numbers at depth other than 2. Char was '" << char( input_ss.peek() ) << "'.";
				error_return = err_ss.str();
				return result;
			}
			float value = 0;
			input_ss >> value;
			if( !!input_ss ) {
				current_vector.push_back( value );
			}
			break;
		}
	}
	if( depth != 0 ) {
		error_return = "Unterminated vector string.";
	} else {
		error_return = "";
	}
	return result;
}

} /* namespace uchile_util */
#endif /* PARAMETERSERVERWRAPPER_H_ */
