/* Nelder-Mead implementation frmo https://github.com/matteotiziano/nelder-mead
 * MIT Licence. Copyright (c) 2015 Matteo Maggioni
 * 
 * TODO: rewrite to not use dynamic memory, because JPL.
 */

void nelder_mead(double *x0, int n, optimset_t optimset, point_t *solution, void *cost_fun_args) {
    
    int        i, j;
    int        iter_count, eval_count;
    int        shrink;
    double     *x_bar;
    double     *x_r, *x_e, *x_c;
    double     fx_r, fx_e, fx_c;
    point_t    *points;
    
    x_bar      = (double*)MALLOC(n*sizeof(double));
    x_r        = (double*)MALLOC(n*sizeof(double));
    x_e        = (double*)MALLOC(n*sizeof(double));
    x_c        = (double*)MALLOC(n*sizeof(double));
    points     = (point_t*)MALLOC((n+1)*sizeof(point_t));
    
    iter_count = 0;
    eval_count = 0;
    
    // initial simplex
    for(i=0; i<n+1; i++) {
        points[i].x = (double*)MALLOC(n*sizeof(double));
        for(j=0; j<n; j++) {
            points[i].x[j] = (i-1==j)? ( x0[j]!=0? 1.05*x0[j] : 0.00025 ) : x0[j];
        }
        points[i].fx = cost_fun(n, points[i].x, cost_fun_args);
        eval_count++;
    }
    qsort((void *)points, n+1, sizeof(point_t), compare);
    get_centroid(n, points, x_bar);
    iter_count++;
    
    // continue minimization until stop conditions are met
    while(continue_minimization(n, points, eval_count, iter_count, optimset)) {
        shrink = 0;
        
        if(optimset.verbose) {
            PRINTF("Iteration %04d \t", iter_count);
        }
        get_point(n, points[n].x, x_bar, RHO, x_r);
        fx_r = cost_fun(n, x_r, cost_fun_args);
        eval_count++;
        if(fx_r<points[0].fx) {
            get_point(n, points[n].x, x_bar, RHO*CHI, x_e);
            fx_e = cost_fun(n, x_e, cost_fun_args);
            eval_count++;
            if(fx_e<fx_r) {
                // expand
                if(optimset.verbose) {
                    PRINTF("expand \t\t");
                }
                copy_point(n, x_e, points[n].x, fx_e, &(points[n].fx));
            } else {
                // reflect
                if(optimset.verbose) {
                    PRINTF("reflect \t");
                }
                copy_point(n, x_r, points[n].x, fx_r, &(points[n].fx));
            }
        } else {
            if(fx_r<points[n-1].fx) {
                // reflect
                if(optimset.verbose) {
                    PRINTF("reflect \t");
                }
                copy_point(n, x_r, points[n].x, fx_r, &(points[n].fx));
            } else {
                if(fx_r<points[n].fx) {
                    get_point(n, points[n].x, x_bar, RHO*GAMMA, x_c);
                    fx_c = cost_fun(n, x_c, cost_fun_args);
                    eval_count++;
                    if(fx_c<=fx_r) {
                        // contract outside
                        if(optimset.verbose) {
                            PRINTF("contract out \t");
                        }
                        copy_point(n, x_c, points[n].x, fx_c, &(points[n].fx));
                    } else {
                        // shrink
                        if(optimset.verbose) {
                            PRINTF("shrink \t\t");
                        }
                        shrink = 1;
                    }
                } else {
                    get_point(n, points[n].x, x_bar, -GAMMA, x_c);
                    fx_c = cost_fun(n, x_c, cost_fun_args);
                    eval_count++;
                    if(fx_c<=points[n].fx) {
                        // contract inside
                        if(optimset.verbose) {
                            PRINTF("contract in \t");
                        }
                        copy_point(n, x_c, points[n].x, fx_c, &(points[n].fx));
                    } else {
                        // shrink
                        if(optimset.verbose) {
                            PRINTF("shrink \t\t");
                        }
                        shrink = 1;
                    }
                }
            }
        }
        if(shrink) {
            for(i=1; i<n+1; i++) {
                for(j=0; j<n; j++) {
                    points[i].x[j] = points[0].x[j] + SIGMA*(points[i].x[j]-points[0].x[j]);
                }
                points[i].fx = cost_fun(n, points[i].x, cost_fun_args);
                eval_count++;
            }
            qsort((void *)points, n+1, sizeof(point_t), compare);
        } else {
            for(i=n-1; i>=0 && points[i+1].fx<points[i].fx; i--) {
                swap_points(n, points+(i+1), points+i);
            }
        }
        get_centroid(n, points, x_bar);
//        iter_count++;
//        if(optimset.verbose) {
//            print_min(n, points);
//        }
    }
    
    // save solution in output argument
    solution->x = (double*)MALLOC(n*sizeof(double));
    copy_point(n, points[0].x, solution->x, points[0].fx, &(solution->fx));

    free(x_bar);
    free(x_r);
    free(x_e);
    free(x_c);
    
}


int compare(const void *arg1, const void *arg2) {
    double fx1, fx2;
    fx1 = (((point_t *)arg1)->fx);
    fx2 = (((point_t *)arg2)->fx);
    
    if(fx1==fx2) {
        return 0;
    } else {
        return (fx1<fx2)? -1 : 1;
    }
}

void get_centroid(int n, point_t *points, double *x_bar) {
    int i, j;
    for(j=0; j<n; j++) {
        x_bar[j] = 0;
        for(i=0; i<n; i++) {
            x_bar[j] += points[i].x[j];
        }
        x_bar[j] /= n;
    }
}

double modulus(double x) {
    return (x>0)? x : -x;
}

int continue_minimization(int n, point_t *points, int eval_count, int iter_count, optimset_t optimset) {
    int i,j;
    double condx = -1;
    double condf = -1;
    double temp;
    if(eval_count>optimset.max_eval || iter_count>optimset.max_iter) {
        // stop if #evals or #iters are greater than the max allowed
        return 0;
    }
    for(i=1; i<n+1; i++) {
        temp = modulus(points[0].fx-points[i].fx);
        if(condf<temp) {
            condf = temp;
        }
    }
    for(i=1; i<n+1; i++) {
        for(j=0; j<n; j++) {
            temp = modulus(points[0].x[j]-points[i].x[j]);
            if(condx<temp) {
                condx = temp;
            }
        }
    }
    // continue if both tolx or tolf condition is not met
    return condx>optimset.tolx || condf>optimset.tolf;
    
}

void get_point(int n, double *x, double *x_bar, double coeff, double *x_out) {
    int j;
    for(j=0; j<n; j++) {
        x_out[j] = (1+coeff)*x_bar[j] - coeff*x[j];
    }
}

void copy_point(int n, double *x_from, double *x_to, double fx_from, double *fx_to) {
    int j;
    for(j=0; j<n; j++) {
        x_to[j] = x_from[j];
    }
    *fx_to = fx_from;
}

void swap_points(int n, point_t *p1, point_t *p2) {
    int j;
    double temp;
    for(j=0; j<n; j++) {
        temp     = p1->x[j];
        p1->x[j] = p2->x[j];
        p2->x[j] = temp;
    }
    temp   = p1->fx;
    p1->fx = p2->fx;
    p2->fx = temp;
}

void print_min(int n, point_t *points) {
    int j;
    printf("[ ");
    for(j=0; j<n; j++) {
        printf("%.2f ", points[0].x[j]);
    }
    printf("]\t%.2f \n", points[0].fx);
}
