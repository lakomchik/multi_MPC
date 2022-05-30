#include "mpc_ip.hpp"



namespace MPC
{


    double customMinFunc(const std::vector<double>& x, std::vector<double>& grad, void* data) {
        // Because we wanted a Class
        // without static members, but NLOpt library does not support
        // passing methods of Classes, we use these auxilary functions.
        // podsmotreno s https://github.com/Aniruddha3395/CUDA_experiments/blob/master/src/opt_obj_eigen.cpp
        MPC *c = (MPC *) data;
        return c->myfunc(x,grad);
    }
    
    void customEqConstraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
    {

        MPC *c = (MPC *) f_data;
        return c->multi_constraint(m, result, n, x, grad);
    }


    void MPC::set_q()
    {
        for (int i = 0; i < n_states; ++i)
        {
            for (int j = 0; j < n_states; ++j)
            {
                Q(i,j).val() = 0.;
            }
        }
    
       
        this->R(0,0).val() = 0.;
        this->Q(0,0).val() = 0.;
        this->Q(1,1).val() = 0.5;
        this->Q(2,2).val() = 5.5;
        this->Q(3,3).val() = 0.4;
      
    }

    void MPC::euler_integration(Eigen::Matrix<autodiff::real, n_states, 1> const & curr_pos, Eigen::Matrix<autodiff::real, n_controls, 1> const & curr_controls,
                        Eigen::Matrix <autodiff::real, n_states, 1> & next_pos, double time_step = dt)
    {
        
        double a = mb*l;
        double I0 = Ib + mb*pow(l,2);
        double mt = mb + 2 * mw;
        double m0 = mt + Iw/pow(r,2); 
        
        next_pos(0,0) = curr_pos(0,0) + time_step * curr_pos(1,0);
        next_pos(1,0) = curr_pos(1,0) + time_step * 1/(I0*m0 - pow(a,2) * pow(cos(curr_pos(2,0)),2))*(a*I0 * pow(curr_pos(3,0),2) * sin(curr_pos(2,0)) - pow(a,2) * g * sin(curr_pos(2,0))  * cos(curr_pos(2,0)) + curr_controls(0,0) * ( I0/r + a *cos(curr_pos(2,0))));
        next_pos(2,0) = curr_pos(2,0) + time_step * curr_pos(2,0);
        next_pos(3,0) = curr_pos(3,0) + time_step * 1/(I0*m0 - pow(a,2) * pow(cos(curr_pos(2,0)),2))*(-pow(a,2)*pow(curr_pos(3,0),2) * sin(curr_pos(2,0))*cos(curr_pos(2,0)) + a*m0*g* sin(curr_pos(2,0)) - curr_controls(0,0) * ( m0 + a/r *cos(curr_pos(2,0))));
        
        // Linearized model next state calculating
        
        /*
        next_pos(0,0) = curr_pos(0,0) + time_step * curr_pos(1,0);
        next_pos(1,0) = curr_pos(1,0) + time_step * (-C1 * curr_pos(2,0) + C2 * curr_controls(0,0));
        next_pos(2,0) = curr_pos(2,0) + time_step * curr_pos(3,0);
        next_pos(3,0) = curr_pos(3,0) + time_step * (C3 * curr_pos(2,0) - C4 * curr_controls(0,0));
        */

    }

    

    Eigen::Matrix<autodiff::real,1,1> MPC::multiple_shooting_objective(const Eigen::Matrix<autodiff::real, N*n_controls+(N+1)*n_states,1> & opt_var)
    {
        Eigen::Matrix<autodiff::real, n_states, 1> cur_pos, delta_pos;
        Eigen::Matrix<autodiff::real, n_controls, 1> cur_control, delta_con;
        Eigen::Matrix<autodiff::real, 1, 1> result, step_cost;
        for (int i = 0 ; i < N; ++i)
        {
            /*for(int k = 0; k < n_states; ++k) //setting states
            {
                cur_pos(k,0)= opt_var(N*n_controls+i*n_states+k,0);
            }*/
            cur_pos(0,0) =opt_var(N*n_controls+i*n_states,0);
            cur_pos(1,0) =opt_var(N*n_controls+i*n_states+1,0);
            cur_pos(2,0) =opt_var(N*n_controls+i*n_states+2,0);
            cur_pos(3,0) =opt_var(N*n_controls+i*n_states+3,0);
            /*for(int k = 0; k < n_controls; ++k) //setting controls
            {
                cur_control(k,0) = opt_var(n_controls*i+k,0);
            }*/
            cur_control(0,0) = opt_var(n_controls*i,0);
            delta_pos = cur_pos-reference_path.col(0);
            delta_con = cur_control-reference_controls.col(0);
           
           
            step_cost = delta_pos.transpose()*Q*delta_pos+delta_con.transpose()*R*delta_con;
            //step_cost*=pow(1.05,i);
            
            result(0,0) += step_cost(0,0); //adding step cost function to objective function
        }
        return result;
    }

    Eigen::Matrix<autodiff::real, n_states*(N+1), 1> MPC::multiple_constraints(const Eigen::Matrix<autodiff::real, N*n_controls+(N+1)*n_states,1> & opt_var)
    {
        Eigen::Matrix<autodiff::real, n_states*(N+1),1> result;
        Eigen::Matrix<autodiff::real, n_states, 1> next_state;
        Eigen::Matrix<autodiff::real, n_states, 1> step_state;
        Eigen::Matrix<autodiff::real, n_controls, 1> step_controls;
        for (int i = 0; i < n_states; ++i) //the 0 state constraits
        {
            result(i,0) = states(i,0) - opt_var(N*n_controls+i,0);
        }
        for(int i = n_states; i < n_states*(N+1); ++i)
        {
            result(i,0) = - opt_var(N*n_controls+i);
            
        }
        for(int i = 0; i < N; ++i)
        {
            step_controls(0,0) = opt_var(i*n_controls);
            step_state(0,0) = opt_var(N*n_controls + i*n_states);
            step_state(1,0) = opt_var(N*n_controls + i*n_states + 1);
            step_state(2,0) = opt_var(N*n_controls + i*n_states + 2);
            step_state(3,0) = opt_var(N*n_controls + i*n_states + 3);
            euler_integration(step_state, step_controls, next_state);
            result(n_states+i*n_states,0) += next_state(0,0);
            result(n_states+i*n_states+1,0) += next_state(1,0);
            result(n_states+i*n_states+2,0) += next_state(2,0);
            result(n_states+i*n_states+3,0) += next_state(3,0);
        }
        return result;
    }

    void MPC::set_init_cond (Eigen::Matrix<autodiff::real, n_states, 1> const & init_cond)
    {
        this->init_cond = init_cond;
        states.col(0) = this->init_cond;
    }

    void MPC::get_jacobian(Eigen::Matrix<autodiff::real, N*n_controls+(N+1)*n_states, 1> & vars,
    double & result)
    {
        Eigen::Matrix<autodiff::real, 1, 1> res;
        func_Jacobian = autodiff::jacobian([&](auto x){return multiple_shooting_objective(x);}, wrt(vars), at(vars ), res);
        result = res(0,0).val();
    }

    void MPC::get_constraints_jacobian(Eigen::Matrix<autodiff::real, N*n_controls+(N+1)*n_states, 1> vars,
    Eigen::Matrix<double, (N+1)*n_states, 1> & results)
    {
        Eigen::Matrix<autodiff::real,(N+1)*n_states,1> res;
        constr_Jacobian = autodiff::jacobian([&](auto x){return multiple_constraints(x);},wrt(vars), at(vars), res);
        //std::cout<<"\nStolbcov: "<<Jx.cols()<<"\nRiadov:"<<Jx.rows()<<"\n";
        for(int i = 0; i < (N+1)*n_states; ++i)
        {
            results(i,0) = res(i,0).val();
        }
    }

    void MPC::set_reference(const Eigen::Matrix<autodiff::real, n_states, N>& ref_path, const Eigen::Matrix<autodiff::real, n_controls, N> & ref_con)
    {
        reference_controls = ref_con;
        reference_path = ref_path;
    }








    double MPC::myfunc(const std::vector<double> &x, std::vector<double> & grad)
    {
        
        
        double res;
        Eigen::Matrix<autodiff::real, N*n_controls+(N+1)*n_states, 1> vars;
        for(int i = 0; i< N*n_controls+(N+1)*n_states; ++i)
        {
            vars(i,0) = x[i];
        }
        (*this).get_jacobian(vars, res);
        if(!grad.empty())
        {
            for(int i = 0; i<N*n_controls+(N+1)*n_states;++i)
            {
                grad[i] = func_Jacobian(0,i);
            }
        }
        return res;
    }

    void MPC::multi_constraint(unsigned m, double *result, unsigned n, const double* x, double* grad)
    {
        Eigen::Matrix<double,(N+1)*n_states,1> values;
        Eigen::Matrix<autodiff::real,(N+1)*n_states+N*n_controls,1> vars;
        for(int i =0; i < N*n_controls+(N+1)*n_states; ++i)
        {
            vars(i,0).val() = x[i];
        }
        (*this).get_constraints_jacobian(vars, values);
        if(grad)
        {
            
            for (int i = 0; i < n_states*(N+1); ++i)
            {
                for (int j = 0; j < n_controls*N + n_states*(N+1); ++j)
                {
                    grad[i*(n_controls*N+n_states*(N+1))+j] = constr_Jacobian(i,j);

                }
            }
        }
        for (int i =0; i < n_states*(N+1); ++i)
        {
            result[i] = values(i,0);
        }
        return;
    }



    void MPC::init_nlopt()
    {
        std::vector<double> x_new(N*n_controls + (N+1)*n_states);
        x = x_new;
        for(int i =0; i< N*n_controls + (N+1)*n_states; ++i)
        {
            x[i] = 0.;
        }
        nlopt::opt new_opt(nlopt::LD_SLSQP, N*n_controls + (N+1)*n_states);
        opt = new_opt;
        double T_max, T_min;
        T_max = 5.2; T_min = - T_max;
        std::vector<double> ub(N*n_controls + (N+1)*n_states), lb(N*n_controls + (N+1)*n_states);
        for(int i = 0; i < N; ++i)
        {
            ub[i*n_controls] = T_max;//T_max;
            lb[i*n_controls] = T_min;//T_min;
        }
        for(int i = N*n_controls; i< N*n_controls + (N+1)*n_states; ++i)
        {
            ub[i] = HUGE_VAL;
            lb[i] = -HUGE_VAL;
        }
        std::vector<double> tol_constraint((N+1)*n_states);
        for (int i = 0; i < (N+1)*n_states; ++i)
        {
            tol_constraint[i] = 1e-8;
        }

        opt.set_upper_bounds(ub);
        opt.set_lower_bounds(lb);
        opt.set_min_objective(customMinFunc,this);
        opt.set_xtol_rel(1e-8);
        opt.set_maxtime(dt);
        opt.add_equality_mconstraint(customEqConstraint, this, tol_constraint);

    }



    Eigen::Matrix<double, n_controls, 1> MPC::solve(Eigen::Matrix<double, n_states, 1> init_cond, double & minf)
    {
       // std::vector<double> x(N*n_controls + (N+1)*n_states);
        for(int i =0; i< N*n_controls + (N+1)*n_states; ++i)
        {
            x[i] = 0.;
        }
        Eigen::Matrix <autodiff::real, n_states, 1> init_state;

        
        init_state(0,0).val() = init_cond(0,0);
        init_state(1,0).val() = init_cond(1,0);
        init_state(2,0).val() = init_cond(2,0);
        init_state(3,0).val() = init_cond(3,0);
        (*this).set_init_cond(init_state);
        try{
            nlopt::result result = opt.optimize(x, minf);
          //  std::cout << "found minimum at f(OK) "
          //      << std::setprecision(10) << minf << std::endl;
        }
        catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
        //opt.optimize(x, minf);
        Eigen::Matrix<double, n_controls, 1> result;
        result(0,0) = x[0];
        return result;
    }



    //Same function, but returns predicted states
    Eigen::Matrix<double, n_controls, 1> MPC::solve(Eigen::Matrix<double, n_states, 1> init_cond, std::vector<double> & predicted_states)
    {
        std::vector<double> x(N*n_controls + (N+1)*n_states);
        for(int i =0; i< N*n_controls + (N+1)*n_states; ++i)
        {
            x[i] = 0.;
        }
        Eigen::Matrix <autodiff::real, n_states, 1> init_state;

        
        init_state(0,0).val() = init_cond(0,0);
        init_state(1,0).val() = init_cond(1,0);
        init_state(2,0).val() = init_cond(2,0);
        init_state(3,0).val() = init_cond(3,0);
        (*this).set_init_cond(init_state);
        double minf;
        try{
            nlopt::result result = opt.optimize(x, minf);
         //   std::cout << "found minimum at f(OK) "
       //         << std::setprecision(10) << minf << std::endl;
        }
        catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
        //opt.optimize(x, minf);

        for(int  i = N*n_controls; i < N*n_controls+(N+1)*n_states; ++i)
        {
            predicted_states[i-N*n_controls] = x[i];
        }

        Eigen::Matrix<double, n_controls, 1> result;
        
        result(0,0) = x[0];
        return result;
    }
}




