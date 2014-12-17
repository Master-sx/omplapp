// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/app.h"
#include "SE2MultiRigidBodyPlanning.pypp.hpp"

namespace bp = boost::python;

struct SE2MultiRigidBodyPlanning_wrapper : ompl::app::SE2MultiRigidBodyPlanning, bp::wrapper< ompl::app::SE2MultiRigidBodyPlanning > {

    SE2MultiRigidBodyPlanning_wrapper(ompl::app::SE2MultiRigidBodyPlanning const & arg )
    : ompl::app::SE2MultiRigidBodyPlanning( arg )
      , bp::wrapper< ompl::app::SE2MultiRigidBodyPlanning >(){
        // copy constructor
        
    }

    SE2MultiRigidBodyPlanning_wrapper(unsigned int n )
    : ompl::app::SE2MultiRigidBodyPlanning( n )
      , bp::wrapper< ompl::app::SE2MultiRigidBodyPlanning >(){
        // constructor
    
    }

    virtual ::ompl::base::ScopedState< ompl::base::StateSpace > getDefaultStartState(  ) const  {
        if( bp::override func_getDefaultStartState = this->get_override( "getDefaultStartState" ) )
            return func_getDefaultStartState(  );
        else{
            return this->ompl::app::SE2MultiRigidBodyPlanning::getDefaultStartState(  );
        }
    }
    
    ::ompl::base::ScopedState< ompl::base::StateSpace > default_getDefaultStartState(  ) const  {
        return ompl::app::SE2MultiRigidBodyPlanning::getDefaultStartState( );
    }

    virtual ::ompl::base::ScopedState< ompl::base::StateSpace > getFullStateFromGeometricComponent( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state ) const  {
        if( bp::override func_getFullStateFromGeometricComponent = this->get_override( "getFullStateFromGeometricComponent" ) )
            return func_getFullStateFromGeometricComponent( boost::ref(state) );
        else{
            return this->ompl::app::SE2MultiRigidBodyPlanning::getFullStateFromGeometricComponent( boost::ref(state) );
        }
    }
    
    ::ompl::base::ScopedState< ompl::base::StateSpace > default_getFullStateFromGeometricComponent( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state ) const  {
        return ompl::app::SE2MultiRigidBodyPlanning::getFullStateFromGeometricComponent( boost::ref(state) );
    }

    virtual ::ompl::base::State const * getGeometricComponentStateInternal( ::ompl::base::State const * state, unsigned int index ) const {
        if( bp::override func_getGeometricComponentStateInternal = this->get_override( "getGeometricComponentStateInternal" ) )
            return func_getGeometricComponentStateInternal( boost::python::ptr(state), index );
        else{
            return this->ompl::app::SE2MultiRigidBodyPlanning::getGeometricComponentStateInternal( boost::python::ptr(state), index );
        }
    }
    
    virtual ::ompl::base::State const * default_getGeometricComponentStateInternal( ::ompl::base::State const * state, unsigned int index ) const {
        return ompl::app::SE2MultiRigidBodyPlanning::getGeometricComponentStateInternal( boost::python::ptr(state), index );
    }

    virtual unsigned int getRobotCount(  ) const  {
        if( bp::override func_getRobotCount = this->get_override( "getRobotCount" ) )
            return func_getRobotCount(  );
        else{
            return this->ompl::app::SE2MultiRigidBodyPlanning::getRobotCount(  );
        }
    }
    
    unsigned int default_getRobotCount(  ) const  {
        return ompl::app::SE2MultiRigidBodyPlanning::getRobotCount( );
    }

    virtual void inferEnvironmentBounds(  ) {
        if( bp::override func_inferEnvironmentBounds = this->get_override( "inferEnvironmentBounds" ) )
            func_inferEnvironmentBounds(  );
        else{
            this->ompl::app::SE2MultiRigidBodyPlanning::inferEnvironmentBounds(  );
        }
    }
    
    void default_inferEnvironmentBounds(  ) {
        ompl::app::SE2MultiRigidBodyPlanning::inferEnvironmentBounds( );
    }

    virtual void inferProblemDefinitionBounds(  ) {
        if( bp::override func_inferProblemDefinitionBounds = this->get_override( "inferProblemDefinitionBounds" ) )
            func_inferProblemDefinitionBounds(  );
        else{
            this->ompl::app::SE2MultiRigidBodyPlanning::inferProblemDefinitionBounds(  );
        }
    }
    
    void default_inferProblemDefinitionBounds(  ) {
        ompl::app::SE2MultiRigidBodyPlanning::inferProblemDefinitionBounds( );
    }

    virtual bool isSelfCollisionEnabled(  ) const  {
        if( bp::override func_isSelfCollisionEnabled = this->get_override( "isSelfCollisionEnabled" ) )
            return func_isSelfCollisionEnabled(  );
        else{
            return this->ompl::app::SE2MultiRigidBodyPlanning::isSelfCollisionEnabled(  );
        }
    }
    
    bool default_isSelfCollisionEnabled(  ) const  {
        return ompl::app::SE2MultiRigidBodyPlanning::isSelfCollisionEnabled( );
    }

    virtual void clear(  ) {
        if( bp::override func_clear = this->get_override( "clear" ) )
            func_clear(  );
        else{
            this->ompl::geometric::SimpleSetup::clear(  );
        }
    }
    
    void default_clear(  ) {
        ompl::geometric::SimpleSetup::clear( );
    }

    void computeGeometrySpecification(  ){
        ompl::app::RigidBodyGeometry::computeGeometrySpecification(  );
    }

    virtual ::ompl::base::ScopedState< ompl::base::StateSpace > getGeometricComponentState( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state, unsigned int index ) const  {
        if( bp::override func_getGeometricComponentState = this->get_override( "getGeometricComponentState" ) )
            return func_getGeometricComponentState( boost::ref(state), index );
        else{
            return this->ompl::app::AppBase< (ompl::app::AppType)0 >::getGeometricComponentState( boost::ref(state), index );
        }
    }
    
    ::ompl::base::ScopedState< ompl::base::StateSpace > default_getGeometricComponentState( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state, unsigned int index ) const  {
        return ompl::app::AppBase< (ompl::app::AppType)0 >::getGeometricComponentState( boost::ref(state), index );
    }

    virtual void setup(  ) {
        if( bp::override func_setup = this->get_override( "setup" ) )
            func_setup(  );
        else{
            this->ompl::app::AppBase< (ompl::app::AppType)0 >::setup(  );
        }
    }
    
    void default_setup(  ) {
        ompl::app::AppBase< (ompl::app::AppType)0 >::setup( );
    }

    virtual ::ompl::base::PlannerStatus solve( double time=1.0e+0 ) {
        if( bp::override func_solve = this->get_override( "solve" ) )
            return func_solve( time );
        else{
            return this->ompl::geometric::SimpleSetup::solve( time );
        }
    }
    
    ::ompl::base::PlannerStatus default_solve( double time=1.0e+0 ) {
        return ompl::geometric::SimpleSetup::solve( time );
    }

    virtual ::ompl::base::PlannerStatus solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        if( bp::override func_solve = this->get_override( "solve" ) )
            return func_solve( boost::ref(ptc) );
        else{
            return this->ompl::geometric::SimpleSetup::solve( boost::ref(ptc) );
        }
    }
    
    ::ompl::base::PlannerStatus default_solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        return ompl::geometric::SimpleSetup::solve( boost::ref(ptc) );
    }

};

void register_SE2MultiRigidBodyPlanning_class(){

    { //::ompl::app::SE2MultiRigidBodyPlanning
        typedef bp::class_< SE2MultiRigidBodyPlanning_wrapper, bp::bases< ompl::app::AppBase< (ompl::app::AppType)0 > > > SE2MultiRigidBodyPlanning_exposer_t;
        SE2MultiRigidBodyPlanning_exposer_t SE2MultiRigidBodyPlanning_exposer = SE2MultiRigidBodyPlanning_exposer_t( "SE2MultiRigidBodyPlanning", bp::init< unsigned int >(( bp::arg("n") )) );
        bp::scope SE2MultiRigidBodyPlanning_scope( SE2MultiRigidBodyPlanning_exposer );
        bp::implicitly_convertible< unsigned int, ompl::app::SE2MultiRigidBodyPlanning >();
        { //::ompl::app::SE2MultiRigidBodyPlanning::getDefaultStartState
        
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( ::ompl::app::SE2MultiRigidBodyPlanning::*getDefaultStartState_function_type)(  ) const;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( SE2MultiRigidBodyPlanning_wrapper::*default_getDefaultStartState_function_type)(  ) const;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "getDefaultStartState"
                , getDefaultStartState_function_type(&::ompl::app::SE2MultiRigidBodyPlanning::getDefaultStartState)
                , default_getDefaultStartState_function_type(&SE2MultiRigidBodyPlanning_wrapper::default_getDefaultStartState) );
        
        }
        { //::ompl::app::SE2MultiRigidBodyPlanning::getFullStateFromGeometricComponent
        
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( ::ompl::app::SE2MultiRigidBodyPlanning::*getFullStateFromGeometricComponent_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const & ) const;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( SE2MultiRigidBodyPlanning_wrapper::*default_getFullStateFromGeometricComponent_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const & ) const;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "getFullStateFromGeometricComponent"
                , getFullStateFromGeometricComponent_function_type(&::ompl::app::SE2MultiRigidBodyPlanning::getFullStateFromGeometricComponent)
                , default_getFullStateFromGeometricComponent_function_type(&SE2MultiRigidBodyPlanning_wrapper::default_getFullStateFromGeometricComponent)
                , ( bp::arg("state") ) );
        
        }
        { //::ompl::app::SE2MultiRigidBodyPlanning::getGeometricComponentStateInternal
        
            typedef ::ompl::base::State const * ( SE2MultiRigidBodyPlanning_wrapper::*getGeometricComponentStateInternal_function_type)( ::ompl::base::State const *,unsigned int ) const;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "getGeometricComponentStateInternal"
                , getGeometricComponentStateInternal_function_type( &SE2MultiRigidBodyPlanning_wrapper::default_getGeometricComponentStateInternal )
                , ( bp::arg("state"), bp::arg("index") )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::app::SE2MultiRigidBodyPlanning::getGeometricComponentStateSpace
        
            typedef ::ompl::base::StateSpacePtr const & ( ::ompl::app::SE2MultiRigidBodyPlanning::*getGeometricComponentStateSpace_function_type)( unsigned int ) const;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "getGeometricComponentStateSpace"
                , getGeometricComponentStateSpace_function_type(&::ompl::app::SE2MultiRigidBodyPlanning::getGeometricComponentStateSpace)
                , ( bp::arg("index") )
                , bp::return_value_policy< bp::copy_const_reference >() );
        
        }
        { //::ompl::app::SE2MultiRigidBodyPlanning::getGeometricComponentStateSpace
        
            typedef ::ompl::base::StateSpacePtr const & ( ::ompl::app::SE2MultiRigidBodyPlanning::*getGeometricComponentStateSpace_function_type)(  ) const;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "getGeometricComponentStateSpace"
                , getGeometricComponentStateSpace_function_type(&::ompl::app::SE2MultiRigidBodyPlanning::getGeometricComponentStateSpace)
                , bp::return_value_policy< bp::copy_const_reference >() );
        
        }
        { //::ompl::app::SE2MultiRigidBodyPlanning::getRobotCount
        
            typedef unsigned int ( ::ompl::app::SE2MultiRigidBodyPlanning::*getRobotCount_function_type)(  ) const;
            typedef unsigned int ( SE2MultiRigidBodyPlanning_wrapper::*default_getRobotCount_function_type)(  ) const;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "getRobotCount"
                , getRobotCount_function_type(&::ompl::app::SE2MultiRigidBodyPlanning::getRobotCount)
                , default_getRobotCount_function_type(&SE2MultiRigidBodyPlanning_wrapper::default_getRobotCount) );
        
        }
        { //::ompl::app::SE2MultiRigidBodyPlanning::inferEnvironmentBounds
        
            typedef void ( ::ompl::app::SE2MultiRigidBodyPlanning::*inferEnvironmentBounds_function_type)(  ) ;
            typedef void ( SE2MultiRigidBodyPlanning_wrapper::*default_inferEnvironmentBounds_function_type)(  ) ;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "inferEnvironmentBounds"
                , inferEnvironmentBounds_function_type(&::ompl::app::SE2MultiRigidBodyPlanning::inferEnvironmentBounds)
                , default_inferEnvironmentBounds_function_type(&SE2MultiRigidBodyPlanning_wrapper::default_inferEnvironmentBounds) );
        
        }
        { //::ompl::app::SE2MultiRigidBodyPlanning::inferProblemDefinitionBounds
        
            typedef void ( ::ompl::app::SE2MultiRigidBodyPlanning::*inferProblemDefinitionBounds_function_type)(  ) ;
            typedef void ( SE2MultiRigidBodyPlanning_wrapper::*default_inferProblemDefinitionBounds_function_type)(  ) ;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "inferProblemDefinitionBounds"
                , inferProblemDefinitionBounds_function_type(&::ompl::app::SE2MultiRigidBodyPlanning::inferProblemDefinitionBounds)
                , default_inferProblemDefinitionBounds_function_type(&SE2MultiRigidBodyPlanning_wrapper::default_inferProblemDefinitionBounds) );
        
        }
        { //::ompl::app::SE2MultiRigidBodyPlanning::isSelfCollisionEnabled
        
            typedef bool ( ::ompl::app::SE2MultiRigidBodyPlanning::*isSelfCollisionEnabled_function_type)(  ) const;
            typedef bool ( SE2MultiRigidBodyPlanning_wrapper::*default_isSelfCollisionEnabled_function_type)(  ) const;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "isSelfCollisionEnabled"
                , isSelfCollisionEnabled_function_type(&::ompl::app::SE2MultiRigidBodyPlanning::isSelfCollisionEnabled)
                , default_isSelfCollisionEnabled_function_type(&SE2MultiRigidBodyPlanning_wrapper::default_isSelfCollisionEnabled) );
        
        }
        { //::ompl::app::RigidBodyGeometry::computeGeometrySpecification
        
            typedef void ( SE2MultiRigidBodyPlanning_wrapper::*computeGeometrySpecification_function_type)(  ) ;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "computeGeometrySpecification"
                , computeGeometrySpecification_function_type( &SE2MultiRigidBodyPlanning_wrapper::computeGeometrySpecification ) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)0 >::getGeometricComponentState
        
            typedef ompl::app::SE2MultiRigidBodyPlanning exported_class_t;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( exported_class_t::*getGeometricComponentState_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const &,unsigned int ) const;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( SE2MultiRigidBodyPlanning_wrapper::*default_getGeometricComponentState_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const &,unsigned int ) const;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "getGeometricComponentState"
                , getGeometricComponentState_function_type(&::ompl::app::AppBase< (ompl::app::AppType)0 >::getGeometricComponentState)
                , default_getGeometricComponentState_function_type(&SE2MultiRigidBodyPlanning_wrapper::default_getGeometricComponentState)
                , ( bp::arg("state"), bp::arg("index") ) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)0 >::setup
        
            typedef ompl::app::SE2MultiRigidBodyPlanning exported_class_t;
            typedef void ( exported_class_t::*setup_function_type)(  ) ;
            typedef void ( SE2MultiRigidBodyPlanning_wrapper::*default_setup_function_type)(  ) ;
            
            SE2MultiRigidBodyPlanning_exposer.def( 
                "setup"
                , setup_function_type(&::ompl::app::AppBase< (ompl::app::AppType)0 >::setup)
                , default_setup_function_type(&SE2MultiRigidBodyPlanning_wrapper::default_setup) );
        
        }
        SE2MultiRigidBodyPlanning_exposer.def("solve", (::ompl::base::PlannerStatus(::ompl::app::SE2MultiRigidBodyPlanning::*)( double ))(&::ompl::app::SE2MultiRigidBodyPlanning::solve), (bp::arg("solveTime")) );
        SE2MultiRigidBodyPlanning_exposer.def("solve", (::ompl::base::PlannerStatus(::ompl::app::SE2MultiRigidBodyPlanning::*)( const ompl::base::PlannerTerminationCondition& ))(&::ompl::app::SE2MultiRigidBodyPlanning::solve), (bp::arg("ptc")) );
        SE2MultiRigidBodyPlanning_exposer.def("clear", &SE2MultiRigidBodyPlanning_wrapper::clear);
        SE2MultiRigidBodyPlanning_exposer.def("getStateSpace", &::ompl::geometric::SimpleSetup::getStateSpace, bp::return_value_policy< bp::copy_const_reference >());
    }

}
