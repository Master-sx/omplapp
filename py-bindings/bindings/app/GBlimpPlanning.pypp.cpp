// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/app.h"
#include "GBlimpPlanning.pypp.hpp"

namespace bp = boost::python;

struct GBlimpPlanning_wrapper : ompl::app::GBlimpPlanning, bp::wrapper< ompl::app::GBlimpPlanning > {

    GBlimpPlanning_wrapper( )
    : ompl::app::GBlimpPlanning( )
      , bp::wrapper< ompl::app::GBlimpPlanning >(){
        // null constructor
    
    }

    virtual void clear(  ) {
        if( bp::override func_clear = this->get_override( "clear" ) )
            func_clear(  );
        else{
            this->ompl::control::SimpleSetup::clear(  );
        }
    }
    
    void default_clear(  ) {
        ompl::control::SimpleSetup::clear( );
    }

    void computeGeometrySpecification(  ){
        ompl::app::RigidBodyGeometry::computeGeometrySpecification(  );
    }

    static ::ompl::control::ControlSpacePtr constructControlSpace(  ){
        return ompl::app::BlimpPlanning::constructControlSpace(  );
    }

    static ::ompl::base::StateSpacePtr constructStateSpace(  ){
        return ompl::app::BlimpPlanning::constructStateSpace(  );
    }

    virtual ::ompl::base::ScopedState< ompl::base::StateSpace > getDefaultStartState(  ) const  {
        if( bp::override func_getDefaultStartState = this->get_override( "getDefaultStartState" ) )
            return func_getDefaultStartState(  );
        else{
            return this->ompl::app::BlimpPlanning::getDefaultStartState(  );
        }
    }
    
    ::ompl::base::ScopedState< ompl::base::StateSpace > default_getDefaultStartState(  ) const  {
        return ompl::app::BlimpPlanning::getDefaultStartState( );
    }

    virtual ::ompl::base::ScopedState< ompl::base::StateSpace > getFullStateFromGeometricComponent( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state ) const  {
        if( bp::override func_getFullStateFromGeometricComponent = this->get_override( "getFullStateFromGeometricComponent" ) )
            return func_getFullStateFromGeometricComponent( boost::ref(state) );
        else{
            return this->ompl::app::BlimpPlanning::getFullStateFromGeometricComponent( boost::ref(state) );
        }
    }
    
    ::ompl::base::ScopedState< ompl::base::StateSpace > default_getFullStateFromGeometricComponent( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state ) const  {
        return ompl::app::BlimpPlanning::getFullStateFromGeometricComponent( boost::ref(state) );
    }

    virtual ::ompl::base::ScopedState< ompl::base::StateSpace > getGeometricComponentState( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state, unsigned int index ) const  {
        if( bp::override func_getGeometricComponentState = this->get_override( "getGeometricComponentState" ) )
            return func_getGeometricComponentState( boost::ref(state), index );
        else{
            return this->ompl::app::AppBase< (ompl::app::AppType)1 >::getGeometricComponentState( boost::ref(state), index );
        }
    }
    
    ::ompl::base::ScopedState< ompl::base::StateSpace > default_getGeometricComponentState( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state, unsigned int index ) const  {
        return ompl::app::AppBase< (ompl::app::AppType)1 >::getGeometricComponentState( boost::ref(state), index );
    }

    virtual ::ompl::base::State const * getGeometricComponentStateInternal( ::ompl::base::State const * state, unsigned int arg1 ) const {
        if( bp::override func_getGeometricComponentStateInternal = this->get_override( "getGeometricComponentStateInternal" ) )
            return func_getGeometricComponentStateInternal( boost::python::ptr(state), arg1 );
        else{
            return this->ompl::app::BlimpPlanning::getGeometricComponentStateInternal( boost::python::ptr(state), arg1 );
        }
    }
    
    virtual ::ompl::base::State const * default_getGeometricComponentStateInternal( ::ompl::base::State const * state, unsigned int arg1 ) const {
        return ompl::app::BlimpPlanning::getGeometricComponentStateInternal( boost::python::ptr(state), arg1 );
    }

    virtual unsigned int getRobotCount(  ) const  {
        if( bp::override func_getRobotCount = this->get_override( "getRobotCount" ) )
            return func_getRobotCount(  );
        else{
            return this->ompl::app::BlimpPlanning::getRobotCount(  );
        }
    }
    
    unsigned int default_getRobotCount(  ) const  {
        return ompl::app::BlimpPlanning::getRobotCount( );
    }

    virtual void inferEnvironmentBounds(  ) {
        if( bp::override func_inferEnvironmentBounds = this->get_override( "inferEnvironmentBounds" ) )
            func_inferEnvironmentBounds(  );
        else{
            this->ompl::app::AppBase< (ompl::app::AppType)1 >::inferEnvironmentBounds(  );
        }
    }
    
    void default_inferEnvironmentBounds(  ) {
        ompl::app::AppBase< (ompl::app::AppType)1 >::inferEnvironmentBounds( );
    }

    virtual void inferProblemDefinitionBounds(  ) {
        if( bp::override func_inferProblemDefinitionBounds = this->get_override( "inferProblemDefinitionBounds" ) )
            func_inferProblemDefinitionBounds(  );
        else{
            this->ompl::app::AppBase< (ompl::app::AppType)1 >::inferProblemDefinitionBounds(  );
        }
    }
    
    void default_inferProblemDefinitionBounds(  ) {
        ompl::app::AppBase< (ompl::app::AppType)1 >::inferProblemDefinitionBounds( );
    }

    virtual bool isSelfCollisionEnabled(  ) const  {
        if( bp::override func_isSelfCollisionEnabled = this->get_override( "isSelfCollisionEnabled" ) )
            return func_isSelfCollisionEnabled(  );
        else{
            return this->ompl::app::BlimpPlanning::isSelfCollisionEnabled(  );
        }
    }
    
    bool default_isSelfCollisionEnabled(  ) const  {
        return ompl::app::BlimpPlanning::isSelfCollisionEnabled( );
    }

    virtual void ode( ::std::vector< double > const & q, ::ompl::control::Control const * ctrl, ::std::vector< double > & qdot ){
        if( bp::override func_ode = this->get_override( "ode" ) )
            func_ode( boost::ref(q), boost::python::ptr(ctrl), boost::ref(qdot) );
        else{
            this->ompl::app::BlimpPlanning::ode( boost::ref(q), boost::python::ptr(ctrl), boost::ref(qdot) );
        }
    }
    
    virtual void default_ode( ::std::vector< double > const & q, ::ompl::control::Control const * ctrl, ::std::vector< double > & qdot ){
        ompl::app::BlimpPlanning::ode( boost::ref(q), boost::python::ptr(ctrl), boost::ref(qdot) );
    }

    void postPropagate( ::ompl::base::State const * state, ::ompl::control::Control const * control, double const duration, ::ompl::base::State * result ){
        ompl::app::BlimpPlanning::postPropagate( boost::python::ptr(state), boost::python::ptr(control), duration, boost::python::ptr(result) );
    }

    virtual void setDefaultBounds(  ) {
        if( bp::override func_setDefaultBounds = this->get_override( "setDefaultBounds" ) )
            func_setDefaultBounds(  );
        else{
            this->ompl::app::BlimpPlanning::setDefaultBounds(  );
        }
    }
    
    void default_setDefaultBounds(  ) {
        ompl::app::BlimpPlanning::setDefaultBounds( );
    }

    virtual void setup(  ) {
        if( bp::override func_setup = this->get_override( "setup" ) )
            func_setup(  );
        else{
            this->ompl::app::AppBase< (ompl::app::AppType)1 >::setup(  );
        }
    }
    
    void default_setup(  ) {
        ompl::app::AppBase< (ompl::app::AppType)1 >::setup( );
    }

    virtual ::ompl::base::PlannerStatus solve( double time=1.0e+0 ) {
        if( bp::override func_solve = this->get_override( "solve" ) )
            return func_solve( time );
        else{
            return this->ompl::control::SimpleSetup::solve( time );
        }
    }
    
    ::ompl::base::PlannerStatus default_solve( double time=1.0e+0 ) {
        return ompl::control::SimpleSetup::solve( time );
    }

    virtual ::ompl::base::PlannerStatus solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        if( bp::override func_solve = this->get_override( "solve" ) )
            return func_solve( boost::ref(ptc) );
        else{
            return this->ompl::control::SimpleSetup::solve( boost::ref(ptc) );
        }
    }
    
    ::ompl::base::PlannerStatus default_solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        return ompl::control::SimpleSetup::solve( boost::ref(ptc) );
    }

};

void register_GBlimpPlanning_class(){

    { //::ompl::app::GBlimpPlanning
        typedef bp::class_< GBlimpPlanning_wrapper, bp::bases< ompl::app::BlimpPlanning, ompl::app::RenderGeometry >, boost::noncopyable > GBlimpPlanning_exposer_t;
        GBlimpPlanning_exposer_t GBlimpPlanning_exposer = GBlimpPlanning_exposer_t( "GBlimpPlanning", bp::init< >() );
        bp::scope GBlimpPlanning_scope( GBlimpPlanning_exposer );
        { //::ompl::app::RigidBodyGeometry::computeGeometrySpecification
        
            typedef void ( GBlimpPlanning_wrapper::*computeGeometrySpecification_function_type)(  ) ;
            
            GBlimpPlanning_exposer.def( 
                "computeGeometrySpecification"
                , computeGeometrySpecification_function_type( &GBlimpPlanning_wrapper::computeGeometrySpecification ) );
        
        }
        { //::ompl::app::BlimpPlanning::constructControlSpace
        
            typedef ::ompl::control::ControlSpacePtr ( *constructControlSpace_function_type )(  );
            
            GBlimpPlanning_exposer.def( 
                "constructControlSpace"
                , constructControlSpace_function_type( &GBlimpPlanning_wrapper::constructControlSpace ) );
        
        }
        { //::ompl::app::BlimpPlanning::constructStateSpace
        
            typedef ::ompl::base::StateSpacePtr ( *constructStateSpace_function_type )(  );
            
            GBlimpPlanning_exposer.def( 
                "constructStateSpace"
                , constructStateSpace_function_type( &GBlimpPlanning_wrapper::constructStateSpace ) );
        
        }
        { //::ompl::app::BlimpPlanning::getDefaultStartState
        
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( ::ompl::app::BlimpPlanning::*getDefaultStartState_function_type)(  ) const;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( GBlimpPlanning_wrapper::*default_getDefaultStartState_function_type)(  ) const;
            
            GBlimpPlanning_exposer.def( 
                "getDefaultStartState"
                , getDefaultStartState_function_type(&::ompl::app::BlimpPlanning::getDefaultStartState)
                , default_getDefaultStartState_function_type(&GBlimpPlanning_wrapper::default_getDefaultStartState) );
        
        }
        { //::ompl::app::BlimpPlanning::getFullStateFromGeometricComponent
        
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( ::ompl::app::BlimpPlanning::*getFullStateFromGeometricComponent_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const & ) const;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( GBlimpPlanning_wrapper::*default_getFullStateFromGeometricComponent_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const & ) const;
            
            GBlimpPlanning_exposer.def( 
                "getFullStateFromGeometricComponent"
                , getFullStateFromGeometricComponent_function_type(&::ompl::app::BlimpPlanning::getFullStateFromGeometricComponent)
                , default_getFullStateFromGeometricComponent_function_type(&GBlimpPlanning_wrapper::default_getFullStateFromGeometricComponent)
                , ( bp::arg("state") ) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)1 >::getGeometricComponentState
        
            typedef ompl::app::GBlimpPlanning exported_class_t;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( exported_class_t::*getGeometricComponentState_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const &,unsigned int ) const;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( GBlimpPlanning_wrapper::*default_getGeometricComponentState_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const &,unsigned int ) const;
            
            GBlimpPlanning_exposer.def( 
                "getGeometricComponentState"
                , getGeometricComponentState_function_type(&::ompl::app::AppBase< (ompl::app::AppType)1 >::getGeometricComponentState)
                , default_getGeometricComponentState_function_type(&GBlimpPlanning_wrapper::default_getGeometricComponentState)
                , ( bp::arg("state"), bp::arg("index") ) );
        
        }
        { //::ompl::app::BlimpPlanning::getGeometricComponentStateInternal
        
            typedef ::ompl::base::State const * ( GBlimpPlanning_wrapper::*getGeometricComponentStateInternal_function_type)( ::ompl::base::State const *,unsigned int ) const;
            
            GBlimpPlanning_exposer.def( 
                "getGeometricComponentStateInternal"
                , getGeometricComponentStateInternal_function_type( &GBlimpPlanning_wrapper::default_getGeometricComponentStateInternal )
                , ( bp::arg("state"), bp::arg("arg1") )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::app::BlimpPlanning::getGeometricComponentStateSpace
        
            typedef ::ompl::base::StateSpacePtr const & ( ::ompl::app::BlimpPlanning::*getGeometricComponentStateSpace_function_type)(  ) const;
            
            GBlimpPlanning_exposer.def( 
                "getGeometricComponentStateSpace"
                , getGeometricComponentStateSpace_function_type(&::ompl::app::BlimpPlanning::getGeometricComponentStateSpace)
                , bp::return_value_policy< bp::copy_const_reference >() );
        
        }
        { //::ompl::app::BlimpPlanning::getRobotCount
        
            typedef unsigned int ( ::ompl::app::BlimpPlanning::*getRobotCount_function_type)(  ) const;
            typedef unsigned int ( GBlimpPlanning_wrapper::*default_getRobotCount_function_type)(  ) const;
            
            GBlimpPlanning_exposer.def( 
                "getRobotCount"
                , getRobotCount_function_type(&::ompl::app::BlimpPlanning::getRobotCount)
                , default_getRobotCount_function_type(&GBlimpPlanning_wrapper::default_getRobotCount) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)1 >::inferEnvironmentBounds
        
            typedef ompl::app::GBlimpPlanning exported_class_t;
            typedef void ( exported_class_t::*inferEnvironmentBounds_function_type)(  ) ;
            typedef void ( GBlimpPlanning_wrapper::*default_inferEnvironmentBounds_function_type)(  ) ;
            
            GBlimpPlanning_exposer.def( 
                "inferEnvironmentBounds"
                , inferEnvironmentBounds_function_type(&::ompl::app::AppBase< (ompl::app::AppType)1 >::inferEnvironmentBounds)
                , default_inferEnvironmentBounds_function_type(&GBlimpPlanning_wrapper::default_inferEnvironmentBounds) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)1 >::inferProblemDefinitionBounds
        
            typedef ompl::app::GBlimpPlanning exported_class_t;
            typedef void ( exported_class_t::*inferProblemDefinitionBounds_function_type)(  ) ;
            typedef void ( GBlimpPlanning_wrapper::*default_inferProblemDefinitionBounds_function_type)(  ) ;
            
            GBlimpPlanning_exposer.def( 
                "inferProblemDefinitionBounds"
                , inferProblemDefinitionBounds_function_type(&::ompl::app::AppBase< (ompl::app::AppType)1 >::inferProblemDefinitionBounds)
                , default_inferProblemDefinitionBounds_function_type(&GBlimpPlanning_wrapper::default_inferProblemDefinitionBounds) );
        
        }
        { //::ompl::app::BlimpPlanning::isSelfCollisionEnabled
        
            typedef bool ( ::ompl::app::BlimpPlanning::*isSelfCollisionEnabled_function_type)(  ) const;
            typedef bool ( GBlimpPlanning_wrapper::*default_isSelfCollisionEnabled_function_type)(  ) const;
            
            GBlimpPlanning_exposer.def( 
                "isSelfCollisionEnabled"
                , isSelfCollisionEnabled_function_type(&::ompl::app::BlimpPlanning::isSelfCollisionEnabled)
                , default_isSelfCollisionEnabled_function_type(&GBlimpPlanning_wrapper::default_isSelfCollisionEnabled) );
        
        }
        { //::ompl::app::BlimpPlanning::ode
        
            typedef void ( GBlimpPlanning_wrapper::*ode_function_type)( ::std::vector< double > const &,::ompl::control::Control const *,::std::vector< double > & ) ;
            
            GBlimpPlanning_exposer.def( 
                "ode"
                , ode_function_type( &GBlimpPlanning_wrapper::default_ode )
                , ( bp::arg("q"), bp::arg("ctrl"), bp::arg("qdot") ) );
        
        }
        { //::ompl::app::BlimpPlanning::postPropagate
        
            typedef void ( GBlimpPlanning_wrapper::*postPropagate_function_type)( ::ompl::base::State const *,::ompl::control::Control const *,double const,::ompl::base::State * ) ;
            
            GBlimpPlanning_exposer.def( 
                "postPropagate"
                , postPropagate_function_type( &GBlimpPlanning_wrapper::postPropagate )
                , ( bp::arg("state"), bp::arg("control"), bp::arg("duration"), bp::arg("result") ) );
        
        }
        { //::ompl::app::BlimpPlanning::setDefaultBounds
        
            typedef void ( ::ompl::app::BlimpPlanning::*setDefaultBounds_function_type)(  ) ;
            typedef void ( GBlimpPlanning_wrapper::*default_setDefaultBounds_function_type)(  ) ;
            
            GBlimpPlanning_exposer.def( 
                "setDefaultBounds"
                , setDefaultBounds_function_type(&::ompl::app::BlimpPlanning::setDefaultBounds)
                , default_setDefaultBounds_function_type(&GBlimpPlanning_wrapper::default_setDefaultBounds) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)1 >::setup
        
            typedef ompl::app::GBlimpPlanning exported_class_t;
            typedef void ( exported_class_t::*setup_function_type)(  ) ;
            typedef void ( GBlimpPlanning_wrapper::*default_setup_function_type)(  ) ;
            
            GBlimpPlanning_exposer.def( 
                "setup"
                , setup_function_type(&::ompl::app::AppBase< (ompl::app::AppType)1 >::setup)
                , default_setup_function_type(&GBlimpPlanning_wrapper::default_setup) );
        
        }
        GBlimpPlanning_exposer.staticmethod( "constructControlSpace" );
        GBlimpPlanning_exposer.staticmethod( "constructStateSpace" );
        GBlimpPlanning_exposer.def("solve", (::ompl::base::PlannerStatus(::ompl::app::GBlimpPlanning::*)( double ))(&::ompl::app::GBlimpPlanning::solve), (bp::arg("solveTime")) );
        GBlimpPlanning_exposer.def("solve", (::ompl::base::PlannerStatus(::ompl::app::GBlimpPlanning::*)( const ompl::base::PlannerTerminationCondition& ))(&::ompl::app::GBlimpPlanning::solve), (bp::arg("ptc")) );
        GBlimpPlanning_exposer.def("clear", &GBlimpPlanning_wrapper::clear);
        GBlimpPlanning_exposer.def("getStateSpace", &::ompl::control::SimpleSetup::getStateSpace, bp::return_value_policy< bp::copy_const_reference >());
    }

}
