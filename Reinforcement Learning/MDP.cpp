/*
	This code implements the pseudocode proposed in https://www.cs.cmu.edu/~katef/DeepRLControlCourse/lectures/lecture3_mdp_planning.pdf,
	with modifications to conform to the suggestions made by the instructor.
		For example, policy stabilization check is made at each step opposite to the pseudocode.

	Furthermore, minor modifications might be made like never computing the inner terms those are not allowed by the policy.
	Overall, the code computes all state-value functions upto optimal case by using Dynamic Programming. Synchronized fetching is used at
	the end of each step.

	This code only works with a specific problem, but can be generalized in terms of any input.
	@author: Tahsincan Köse
*/

#include <iostream>
#include <vector>
#include <map>
#include <cmath>

using namespace std;


// This struct is required to hold each action-state-next_state information.
struct sv_info{
	float value;
	map<pair<char,char>,vector<char> > actions;
};

map<pair<char,char>, float> pi; //Policy.
map<char,sv_info> state_value_functions; // state_value_function stores V(s) as well as action-state-next_state information.
map<tuple<char,char,char>,float > transition_table; // Only transition table.
map<pair<char,char>, int> reward_table; // Stores R(S,a) values.


//................................
bool in(char a,vector<char>& v);
bool policy_stable(float gamma);
void MDPInfo();
//............................... Utilization functions
// Hard coded variables specific to the problem in order to ease upcoming instructions.
pair<char,char> x = {'b','A'}; 
pair<char,char> y = {'a','B'};
pair<char,char> z = {'d','B'};
pair<char,char> w = {'e','C'};
pair<char,char> q = {'f','C'};
// .....
bool in(char a, vector<char> & v){
	for(int i=0;i<v.size();i++)
		if(a==v[i]) return true;
	return false;
}

bool policy_stable(float gamma){
	bool stable = true;
	for(auto it=state_value_functions.begin();it!=state_value_functions.end();it++){
		char state = it->first;
		map<pair<char,char>,vector<char> > all_actions = it->second.actions;
		vector<char> max_actions;
		float max_value = 0;
		vector<char> valid_actions;
		for(auto ait  = all_actions.begin();ait!=all_actions.end();ait++){
			char action = ait->first.first;
			char state = ait->first.second;
			float value = 0;
			if(pi[{action,state}]>0.0){
				valid_actions.push_back(action);
				vector<char> possible_next_states= ait->second;
				float inner_term = 0;
				for(int i=0;i<possible_next_states.size();i++){
					char next_state = possible_next_states[i];
					inner_term += transition_table[make_tuple(action,state,next_state)] * state_value_functions[next_state].value;
				}

				value += reward_table[{state,action}] + gamma * (inner_term) ;
				if(value > max_value){
					max_value = value;
					max_actions.clear();
					max_actions.push_back(action);
				}
				else if(value == max_value)
					max_actions.push_back(action);
			}
		}
		if(all_actions.size()==1) continue;
		else{
			int sz = max_actions.size();
			for(int i=0;i<sz;i++)
				pi[{max_actions[i],state}] = 1.0/sz;
			for(int i=0;i<valid_actions.size();i++){
				if(!in(valid_actions[i],max_actions)){
					pi[{valid_actions[i],state}] = 0.0;
					stable = false;
				}
			}
		}
	}
	return stable;
}

void MDPInfo(){
	cout<<"V(A) = "<<state_value_functions['A'].value<<endl;
	cout<<"V(B) = "<<state_value_functions['B'].value<<endl;
	cout<<"V(C) = "<<state_value_functions['C'].value<<endl<<endl;
	cout<<"Policy Table:"<<endl;
	cout<<"PI("<<x.first<<","<<x.second<<") = "<<pi[x]<<endl;
	cout<<"PI("<<y.first<<","<<y.second<<") = "<<pi[y]<<endl;
	cout<<"PI("<<z.first<<","<<z.second<<") = "<<pi[z]<<endl;
	cout<<"PI("<<w.first<<","<<w.second<<") = "<<pi[w]<<endl;
	cout<<"PI("<<q.first<<","<<q.second<<") = "<<pi[q]<<endl<<endl;
}

int main(){
	//.....
	pi[x] = 1.0;
	pi[y] = 0.5; pi[z] = 0.5;
	pi[w] = 0.5; pi[q] = 0.5;
	//......Initialization of equiprobable policy.

	//.....
	map<pair<char,char> ,vector<char> > a,b,c;
	a[x].push_back('B');
	b[y].push_back('A');b[z].push_back('C');
	c[w].push_back('B');c[q].push_back('A');c[q].push_back('C');

	state_value_functions['A'] = {1.0,a};
	state_value_functions['B'] = {1.0,b};
	state_value_functions['C'] = {1.0,c}; 
	//.....Initialization of state_value_functions

	//......
	transition_table[make_tuple('b','A','B')] = 1.0;
	transition_table[make_tuple('a','B','A')] = 1.0;
	transition_table[make_tuple('d','B','C')] = 1.0;
	transition_table[make_tuple('e','C','B')] = 1.0;
	transition_table[make_tuple('f','C','A')] = 0.75; 
	transition_table[make_tuple('f','C','C')] = 0.25;
	//...... Initialization of transition table.

	//.....
	reward_table[{'A','b'}] = -5;
	reward_table[{'B','a'}] = 2;
	reward_table[{'B','d'}] = -4;
	reward_table[{'C','e'}] = -8;
	reward_table[{'C','f'}] = 3;
	//..... Initialization of reward table. Consider it as r(S,a).
	float threshold = 0.05;
	float gamma = 0.4;
	int iteration = 1;
	while(true){
		cout<<"Iteration: "<<iteration++<<endl;
		float max_difference = 0; // We need this to compare with threshold.
		vector<float> V_new;
		for(auto it = state_value_functions.begin();it!=state_value_functions.end();it++){
			float v_old = it->second.value;
			float v = 0;
			/*
			   Fetch all action-state-next_state triples for particular state.
			   Might be seem ambiguous, because we already have the particular state from state_value_function->key.
			   However, since predefined pairs <x,y,z,w,q> exist as pairs, I didn't change the way it works and directly
			   used them as keys. It is not a problem in terms of functionality or memory-usage.
			*/
			map<pair<char,char>,vector<char> > actions = it->second.actions; 
			for(auto ait  = actions.begin();ait!=actions.end();ait++){
				char action = ait->first.first;
				char state = ait->first.second;
				vector<char> possible_next_states= ait->second;
				float inner_term = 0;
				for(int i=0;i<possible_next_states.size();i++){
					char next_state = possible_next_states[i];
					//Use transition table to fetch probability of realization for (action,state,next_state) triple.
					float t = transition_table[make_tuple(action,state,next_state)] * state_value_functions[next_state].value;
					inner_term += t;
				}
				// Finally compute v, which is the new value.
				v += pi[{action,state}] * (reward_table[{state,action}] + gamma * (inner_term)) ;
			}
			V_new.push_back(v);
			max_difference = max(max_difference,fabs(v-v_old));
		}
		/*
			policy_stable() function is not just a condition checker but also effectively adjusts the policy.
			That is, independent from the maximum difference, the policy is changed at each step
		*/
		if(policy_stable(gamma) && max_difference<threshold ){ 

			int i=0;
			for(auto it = state_value_functions.begin();it!=state_value_functions.end();it++,i++)
				it->second.value = V_new[i];
			break;
		}
		/* This is a guard to cover the cases where the process is far from the convergence point.
		   Might be seem as a code repetititon, but since policy_stable() method needs the former values of 
		   state-value functions, it is needed to be designed this way. Maybe an auxiliary data structure to hold former
		   values of state-value functions can be used to eliminate below code block.
		*/ 
		int i=0;
		for(auto it = state_value_functions.begin();it!=state_value_functions.end();it++,i++)
			it->second.value = V_new[i];

		MDPInfo();	
	}
	cout<<"-------------------Optimal State-Value Functions-------------------"<<endl;
	MDPInfo();
	return 0;
}