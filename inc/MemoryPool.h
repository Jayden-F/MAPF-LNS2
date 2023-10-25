#pragma once
#include "SIPPNode.h"

const uint32_t DEFAULT_CHUNK_SIZE = 1024 * 1024; // 1MB

namespace node_pool_ns
{
    static const uint64_t NBS = 8; // node block size; set this >= 8
    static const uint64_t LOG2_NBS = 3;
    static const uint64_t NBS_MASK = 7;
}

class cchunk
{
public:
    cchunk(size_t obj_size, size_t pool_size) : obj_size_(obj_size),
                                                pool_size_(pool_size - (pool_size % obj_size)) // round down
    {
        if (pool_size_ < obj_size_)
        {
            std::cerr << "warthog::mem::cchunk object size < pool size; "
                      << "setting pool size to object size" << std::endl;
            pool_size_ = obj_size_;
        }

        mem_ = new char[pool_size_];
        next_ = mem_;
        max_ = mem_ + pool_size_;

        freed_stack_ = new size_t[(pool_size_ / obj_size)];
        stack_size_ = 0;
    }

    ~cchunk()
    {
        delete[] mem_;
        delete[] freed_stack_;
    }

    inline void
    reclaim()
    {
        next_ = mem_;
        stack_size_ = 0;
    }

    inline char *
    allocate()
    {
        if (next_ < max_)
        {
            char *retval = next_;
            next_ += obj_size_;
            return retval;
        }

        if (stack_size_ > 0)
        {
            --stack_size_;
            return mem_ + freed_stack_[stack_size_];
        }

        return 0;
    }

    inline void
    deallocate(char *addr)
    {
#ifndef NDEBUG
        assert(mem_ >= addr);
        if ((size_t)(addr - mem_) >= pool_size_)
        {
            std::cerr << "err; warthog::mem::cchunk; freeing memory outside"
                         " range of the chunk at addr: "
                      << &mem_ << "\n";
        }
#endif

        freed_stack_[stack_size_] = (size_t)(addr - mem_);
        stack_size_++;
    }

    inline bool
    contains(char *addr)
    {
#ifndef NDEBUG
        assert(mem_ >= addr);
#endif
        if ((unsigned)(addr - mem_) < pool_size_)
        {
            return true;
        }
        return false;
    }

    inline char *
    first_addr()
    {
        return mem_;
    }

    inline size_t
    pool_size()
    {
        return pool_size_;
    }

    inline size_t
    mem()
    {
        size_t bytes = sizeof(*this);
        bytes += sizeof(char) * pool_size_;
        bytes += sizeof(int) * (pool_size_ / obj_size_);
        return bytes;
    }

    void
    print(std::ostream &out)
    {
        out << "warthog::mem::cchunk pool_size: " << pool_size_
            << " obj_size: " << obj_size_ << " freed_stack_ size: " << stack_size_;
    }

private:
    char *mem_;
    char *next_;
    char *max_;

    size_t obj_size_;
    size_t pool_size_;

    // keep a stack of freed objects
    size_t *freed_stack_;
    size_t stack_size_;
};

class cpool
{
public:
    cpool(size_t obj_size, size_t max_chunks) : num_chunks_(0), max_chunks_(max_chunks), obj_size_(obj_size)
    {
        init();
    }

    cpool(size_t obj_size) : num_chunks_(0), max_chunks_(20), obj_size_(obj_size)
    {
        init();
    }

    ~cpool()
    {
        for (size_t i = 0; i < num_chunks_; i++)
        {
            delete chunks_[i];
        }
        delete[] chunks_;
    }

    inline void
    reclaim()
    {
        for (size_t i = 0; i < num_chunks_; i++)
        {
            chunks_[i]->reclaim();
        }
    }

    inline char *
    allocate()
    {
        char *mem_ptr = current_chunk_->allocate();
        if (!mem_ptr)
        {
            // look for space in an existing chunk
            // NB: linear-time search! increase CHUNK_SIZE_ if
            // number of chunks grows too large
            for (unsigned int i = 0; i < num_chunks_; i++)
            {
                mem_ptr = chunks_[i]->allocate();
                if (mem_ptr)
                {
                    current_chunk_ = chunks_[i];
                    return mem_ptr;
                }
            }

            // not enough space in any existing chunk; make a new one
            add_chunk(CHUNK_SIZE_);
            current_chunk_ = chunks_[num_chunks_ - 1];
            mem_ptr = current_chunk_->allocate();
        }
        return mem_ptr;
    }

    inline void
    deallocate(char *addr)
    {
        for (unsigned int i = 0; i < num_chunks_; i++)
        {
            if ((unsigned)(addr - chunks_[i]->first_addr()) < chunks_[i]->pool_size())
            {
                chunks_[i]->deallocate(addr);
                return;
            }
        }
#ifndef NDEBUG
        std::cerr << "err; cpool::free "
                     "tried to free an address not in any chunk!\n";
#endif
    }

    size_t
    mem()
    {
        size_t bytes = 0;
        for (unsigned int i = 0; i < num_chunks_; i++)
        {
            bytes += chunks_[i]->mem();
        }
        bytes += sizeof(cchunk *) * max_chunks_;
        bytes += sizeof(*this);
        return bytes;
    }

    void
    print(std::ostream &out)
    {
        out << "warthog::mem::cpool #chunks: " << num_chunks_
            << " #max_chunks " << max_chunks_ << " obj_size: " << obj_size_;
        out << std::endl;
        for (unsigned int i = 0; i < num_chunks_; i++)
        {
            chunks_[i]->print(out);
            out << std::endl;
        }
    }

private:
    cchunk **chunks_;
    cchunk *current_chunk_;

    size_t num_chunks_;
    size_t max_chunks_;
    size_t obj_size_;
    size_t CHUNK_SIZE_;

    // no copy
    cpool(const cpool &other) {}
    cpool &
    operator=(const cpool &other) { return *this; }

    void
    init()
    {
        // chunk size needs to be at least as big as one object
        CHUNK_SIZE_ = std::max<size_t>(obj_size_, DEFAULT_CHUNK_SIZE);

        chunks_ = new cchunk *[max_chunks_];
        for (int i = 0; i < (int)max_chunks_; i++)
        {
            add_chunk(CHUNK_SIZE_);
        }
        current_chunk_ = chunks_[0];
    }

    void
    add_chunk(size_t pool_size)
    {
        if (num_chunks_ < max_chunks_)
        {
            chunks_[num_chunks_] = new cchunk(obj_size_, pool_size);
            num_chunks_++;
        }
        else
        {
            // make room for a new chunk
            size_t big_max = max_chunks_ * 2;
            cchunk **big_chunks = new cchunk *[big_max];
            for (unsigned int i = 0; i < max_chunks_; i++)
            {
                big_chunks[i] = chunks_[i];
            }
            delete[] chunks_;

            chunks_ = big_chunks;
            max_chunks_ = big_max;

            // finally; add a new chunk
            chunks_[num_chunks_] = new cchunk(obj_size_, pool_size);
            num_chunks_++;
        }
    }
};

class MemoryPool
{
    // every location in the map has a node with id equal to the location
public:
    MemoryPool() : blocks_(0)
    {
        numblocks_ = 0;
        label_ = 0;
    };
    MemoryPool(uint64_t size)
    {
        init(size);
    };

    ~MemoryPool()
    {
        blockspool_->reclaim();
        delete blockspool_;

        for (size_t i = 0; i < numblocks_; i++)
        {
            if (blocks_[i] != 0)
            {
                // std::cerr << "deleting block: "<<i<<std::endl;
                blocks_[i] = 0;
            }
        }
        delete[] blocks_;
    }

    void init(u_int64_t size)
    {
        numblocks_ = ((size) >> node_pool_ns::LOG2_NBS) + 1;
        blocks_ = new SIPPNode *[numblocks_];
        for (size_t i = 0; i < numblocks_; i++)
        {
            blocks_[i] = 0;
        }

        size_t block_sz = node_pool_ns::NBS * sizeof(SIPPNode);
        blockspool_ = new cpool(block_sz, 1);

        label_ = 0;
        ready_ = true;
    }

    bool is_ready()
    {
        return ready_;
    }

    bool has_node(u_int64_t id)
    {
        // if (id >= numblocks_)
        // {
        //     std::cout << "range out of memory pool size " << id << "," << numblocks_ << std::endl;
        //     exit(1);
        // }
        SIPPNode *node = generate(id);
        return node->label == label_ && node->id == id;
    }

    bool is_closed(u_int64_t id)
    {
        // if (id >= numblocks_)
        // {
        //     std::cout << "range out of memory pool size " << id << "," << numblocks_ << std::endl;
        //     exit(1);
        // }
        SIPPNode *node = generate(id);
        if (node->label != label_)
        {
            return false;
        }
        return node->is_closed;
    }
    SIPPNode *get_node(u_int64_t id)
    {
        // if (id >= numblocks_)
        // {
        //     std::cout << "range out of memory pool size " << id << "," << numblocks_ << std::endl;
        //     exit(1);
        // }
        SIPPNode *node = generate(id);
        if (node->label != label_ || node->id == -1)
        {
            std::cout << "error node not generated yet" << std::endl;
            exit(1);
        }
        return node;
    }
    void close_node(u_int64_t id)
    {
        if (id >= numblocks_)
        {
            std::cout << "range out of memory pool size " << id << "," << numblocks_ << std::endl;
            exit(1);
        }
        SIPPNode *node = generate(id);
        if (node->label != label_ || node->id == -1)
        {
            std::cout << "node not generated yet" << std::endl;
            exit(1);
        }
        node->close();
    }

    SIPPNode *generate(u_int64_t node_id)
    {
        u_int64_t block_id = node_id >> node_pool_ns::LOG2_NBS;
        u_int64_t list_id = node_id & node_pool_ns::NBS_MASK;

        // id outside the pool address range
        if (block_id > numblocks_)
        {
            return 0;
        }

        // add a new block of nodes if necessary
        if (!blocks_[block_id])
        {
            // std::cerr << "generating block: "<<block_id<<std::endl;
            blocks_[block_id] = new (blockspool_->allocate()) SIPPNode[node_pool_ns::NBS];

            // initialise memory
            u_int64_t current_id = node_id - list_id;
            for (uint32_t i = 0; i < node_pool_ns::NBS; i += 8)
            {
                new (&blocks_[block_id][i]) SIPPNode(current_id++);
                new (&blocks_[block_id][i + 1]) SIPPNode(current_id++);
                new (&blocks_[block_id][i + 2]) SIPPNode(current_id++);
                new (&blocks_[block_id][i + 3]) SIPPNode(current_id++);
                new (&blocks_[block_id][i + 4]) SIPPNode(current_id++);
                new (&blocks_[block_id][i + 5]) SIPPNode(current_id++);
                new (&blocks_[block_id][i + 6]) SIPPNode(current_id++);
                new (&blocks_[block_id][i + 7]) SIPPNode(current_id++);
            }
        }

        // return the node from its position in the assocated block
        return &(blocks_[block_id][list_id]);
    }

    SIPPNode *generate_node(u_int64_t id, int location, int g_val, int h_val, SIPPNode *parent, int timestep, const SIPPInterval *interval, int interval_index)
    {
        // if (id >= numblocks_)
        // {
        //     std::cout << "range out of memory pool size " << id << "," << numblocks_ << std::endl;
        //     exit(1);
        // }

        SIPPNode *node = generate(id);
        if (node->label == label_ && node->id != -1)
        {
            std::cout << "node already generated " << id << "," << is_ready() << std::endl;

            std::cout << "node already generated " << node->id << std::endl;
            exit(1);
        }
        node->reset();

        if (node->location != location && node->location != -1)
        {
            std::cout << "node location is different " << node->location << "," << location << std::endl;
            exit(1);
        }

        node->reset();
        node->id = id;
        node->label = label_;
        node->location = location;
        node->g_val = g_val;
        node->h_val = h_val;
        node->parent = parent;
        node->timestep = timestep;
        node->interval = interval;
        node->interval_index = interval_index;
        // node->high_expansion = high_expansion;
        // node->collision_v = collision_v;
        // node->num_of_conflicts = num_of_conflicts;

        return node;
    }
    SIPPNode *generate_node(u_int64_t id, SIPPNode &new_node)
    {
        SIPPNode *node = generate(id);

        if (node->location != new_node.location && node->location != -1)
        {
            std::cout << "node location is different " << node->location << "," << new_node.location << std::endl;
            exit(1);
        }
        node->reset();
        node->id = id;
        node->label = label_;
        node->location = new_node.location;
        node->g_val = new_node.g_val;
        node->h_val = new_node.h_val;
        node->parent = new_node.parent;
        node->timestep = new_node.timestep;
        node->interval = new_node.interval;
        node->interval_index = new_node.interval_index;
        // node->collision_v = new_node.collision_v;
        // node->num_of_conflicts = new_node.num_of_conflicts;
        return node;
    }
    SIPPNode *replace_node(u_int64_t id, SIPPNode &new_node)
    {
        SIPPNode *node = generate(id);

        if (node->location != new_node.location && node->location != -1)
        {
            std::cout << "node location is different " << node->location << "," << new_node.location << std::endl;
            exit(1);
        }
        node->id = id;
        node->label = label_;
        node->location = new_node.location;
        node->g_val = new_node.g_val;
        node->h_val = new_node.h_val;
        node->parent = new_node.parent;
        node->timestep = new_node.timestep;
        node->interval = new_node.interval;
        node->interval_index = new_node.interval_index;
        // node->collision_v = new_node.collision_v;
        // node->num_of_conflicts = new_node.num_of_conflicts;
        return node;
    }

    void reset()
    {
        label_++;
    }

    SIPPNode *get_ptr(u_int64_t node_id)
    {
        u_int64_t block_id = node_id >> node_pool_ns::LOG2_NBS;
        u_int64_t list_id = node_id & node_pool_ns::NBS_MASK;

        // id outside the pool address range
        if (block_id > numblocks_)
        {
            return 0;
        }

        if (!blocks_[block_id])
        {
            return 0;
        }
        return &(blocks_[block_id][list_id]);
    }

    size_t mem()
    {
        size_t bytes =
            sizeof(*this) +
            blockspool_->mem() +
            numblocks_ * sizeof(void *);

        return bytes;
    }

private:
    cpool *blockspool_;
    SIPPNode **blocks_;
    int numblocks_;
    int label_;
    bool ready_ = false;
};
