#include "incestimator/TerrainEstimator.h"


template < typename T > TerrainEstimator< T >::TerrainEstimator() {}

/**
 * @brief Initialize
 *
 */
template < typename T > void TerrainEstimator< T >::setup() {
    W_plane_.resize( 4, 3 );
    W_plane_.setZero();
    W_plane_( 0, 0 ) = 1;
    W_plane_( 1, 0 ) = 1;
    W_plane_( 2, 0 ) = 1;
    W_plane_( 3, 0 ) = 1;
    z_feet_.setZero();
    a_plane_.setZero();
    Vec3< float > rpyplane;
    rpyplane.setZero();

    Vec3< float > norpla( -a_plane_( 1 ), -a_plane_( 2 ), 1 );
    Vec3< float > nor2( a_plane_( 2 ), -a_plane_( 1 ), 0 );
    norpla.normalize();
    nor2.normalize();
    Vec3< float > nor3;
    nor3                         = nor2.cross( norpla );
    R_plane_.block( 0, 0, 3, 1 ) = nor3;
    R_plane_.block( 0, 1, 3, 1 ) = nor2;
    R_plane_.block( 0, 2, 3, 1 ) = norpla;
    rpyplane                     = ori::rotationMatrixToRPY( R_plane_ );
    rpyplane( 2 )                = 0;
    R_plane_                     = ori::rpyToRotMat( rpyplane );
}

/**
 * @brief Run terrain estimator
 *
 */
template < typename T > void TerrainEstimator< T >::run() {
    // auto&     quadruped    = *this->state_estimator_data_.leg_controller_data->quadruped;
    // Vec4< T > swing_states = *( this->state_estimator_data_.swing_phase );
    // ( void )swing_states;
    Vec4< T > contact_states = this->_stateEstimatorData.result->contactEstimate;
    Vec3< T > foot_pos_local[ 4 ];
    Vec3< T > foot_pos_world[ 4 ];
    float     trust = 0.15;
    for ( int i = 0; i < 4; ++i ) {
        foot_pos_local[ i ] = legdata->GetHipLocation( i ) + legdata->foot_pos_local[i];
        foot_pos_world[ i ] = attidata->rot_body.transpose() * foot_pos_local[ i ];
        foot_pos_world[ i ] += this->_stateEstimatorData.result->position;
    }

    for ( int foot = 0; foot < 4; foot++ ) {
        // std::numeric_limits<double>::epsilon()
        // if ( swing_states[ foot ] <= std::numeric_limits< float >::epsilon() ) {  // Original of SWING
        if ( contact_states[ foot ] <= 1 - trust && contact_states[ foot ] >= trust ) {
            W_plane_( foot, 1 ) = foot_pos_world[ foot ]( 0 );
            W_plane_( foot, 2 ) = foot_pos_world[ foot ]( 1 );
            z_feet_( foot )     = foot_pos_world[ foot ]( 2 );
            pseudoInverse(W_plane_, 0.001, W_plane_inv_);
            //  smooth transition
            a_plane_ = ( 1 - 0.1 ) * a_plane_ + ( 0.1 ) * W_plane_inv_ * z_feet_;
            // a_plane_=(1-(data.user_parameters->mpc_velocity_filter))*a_plane_+(data.user_parameters->mpc_velocity_filter)*Wplainv*z_feet_;
            // a_plane_ = Wplainv * z_feet_;
            //    std::cout << " test ground prediction : " << foot << ' ' << swing_states[foot] << std::endl;
        }
    }

    Vec3< float > norpla( -a_plane_( 1 ), -a_plane_( 2 ), 1 );
    Vec3< float > nor2( a_plane_( 2 ), -a_plane_( 1 ), 0 );
    norpla.normalize();
    nor2.normalize();
    Vec3< float > nor3;
    nor3                         = nor2.cross( norpla );
    R_plane_.block( 0, 0, 3, 1 ) = nor3;
    R_plane_.block( 0, 1, 3, 1 ) = nor2;
    R_plane_.block( 0, 2, 3, 1 ) = norpla;
    Vec3< float > rpyplane;
    rpyplane      = ori::rotationMatrixToRPY( R_plane_ );
    rpyplane( 2 ) = 0;
    R_plane_      = ori::rpyToRotMat( rpyplane );
    for ( int i = 0; i < 3; i++ ) {
        this->_stateEstimatorData.result->terrainCoefficient( i )        = a_plane_( i );
        this->_stateEstimatorData.result->terrainRotationMatrix( i, 0 ) = R_plane_( i, 0 );
        this->_stateEstimatorData.result->terrainRotationMatrix( i, 1 ) = R_plane_( i, 1 );
        this->_stateEstimatorData.result->terrainRotationMatrix( i, 2 ) = R_plane_( i, 2 );
    }

    // Eigen::Matrix<float, 4, 1> quatdes;
    // Mat3<float> est_terrain_rot_matrix_ = this->_stateEstimatorData.result->terrainRotationMatrix;
    // quatdes = ori::rotationMatrixToQuaternion(est_terrain_rot_matrix_.transpose());
    // Eigen::Matrix<float, 4, 1> quatprev(attidata->quat[0], attidata->quat[1], attidata->quat[2], attidata->quat[3]);
    // // The step below could avoid sign error
    // if ((quatprev - quatdes).norm() > (quatprev + quatdes).norm())
    // {
    //     quatdes = -quatdes;
    // }
    // Vec3<float> rpy_des_with_comp_ = ori::quatToRPY(quatdes);
    // if (fabs(rpy_des_with_comp_[2] - attidata->rpy[2]) > 3.14)
    // {
    //     rpy_des_with_comp_[2] = -rpy_des_with_comp_[2];
    // }
    // std::cout << "rpy:" << "," << rpy_des_with_comp_[0] << "," << rpy_des_with_comp_[1] << std::endl;
}

template class TerrainEstimator< float >;
// template class TerrainEstimator< double >;
