#ifndef LPA_PRORITY_QUEUE_HPP
#define LPA_PRORITY_QUEUE_HPP
#include <map>

namespace arc_dijkstras
{

    /** 
     *  Priority queue with top(), pop(), insert(key-value), remove(value)
     *
     */
    template<typename Priority, typename Element, class Compare=std::less<Priority>>
    class pqueue
    {
        typedef typename std::multimap<Priority, Element, Compare>::iterator iterator;
        
    protected:
        std::multimap<Priority, Element, Compare> priority_queue;
        std::multimap<Element, iterator> reverse_lookup;
        
    public:
        void insert(std::pair<Priority, Element> kv)
        {
            iterator it = priority_queue.insert(kv);
            reverse_lookup.insert({kv.second, it});
        }

        
        bool isEmpty() const 
        {
            return priority_queue.empty();
        }
        

        std::pair<Priority, Element> top()
        {
            return {priority_queue.begin()->first, priority_queue.begin()->second};
        }

        std::pair<Priority, Element> pop()
        {
            auto kv = top();
            remove(kv.second);
            return kv;
        }

        size_t remove(Element elem)
        {
            auto ret = reverse_lookup.equal_range(elem);
            for(auto it=ret.first; it!=ret.second; it++)
            {
                priority_queue.erase(it->second);
            }
            return reverse_lookup.erase(elem);
        }
    };
}


#endif
