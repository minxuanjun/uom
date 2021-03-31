#pragma once

#include <iostream>
#include <string>
#include <iomanip>

#include <Eigen/Core>
#include <glog/logging.h>

#include "uom/utils/yaml_parser.hpp"

struct ParamsBase
{
    ParamsBase() = default;

    virtual ~ParamsBase() = default;

    virtual void parse_yaml(const std::string& file_path) = 0;

    virtual void print() = 0;

protected:

    template <typename TName, typename TValue>
    void print_impl(std::stringstream& out, TName name, TValue value) const
    {
        out.width(kNameWidth);
        out.setf(std::ios::left, std::ios::adjustfield);
        out << name;
        out.width(kValueWidth);
        out.setf(std::ios::right, std::ios::adjustfield);
        out << value << '\n';
    }

    template <typename TName>
    void print_impl(std::stringstream& out, TName name, Eigen::Matrix4d value) const
    {
        static Eigen::IOFormat kCleanFmt{4, 0, ", ", "\n", "\t\t[", "]"};
        out << std::setfill(' ');
        out << name << '\n' << value.format(kCleanFmt) << '\n';
        out << std::setfill('.');
    }

    template <typename TName>
    void print_impl(std::stringstream& out, TName name, Eigen::Vector3d value) const
    {
        static Eigen::IOFormat kCleanFmt{5, 0, ", ", "\n", "[", "]"};

        std::stringstream ss;
        ss << value.transpose().format(kCleanFmt);
        print_impl(out, name, ss.str());
    }

    template <typename TName>
    void print_impl(std::stringstream& out, TName name, bool value) const
    {
        print_impl(out, name, value ? "true":"false");
    }


    template <typename TName, typename TValue>
    void print_impl_recur(std::stringstream& out, TName name, TValue value) const
    {
        print_impl(out, name, value);
    }


    template <typename TName, typename TValue, typename... Args>
    void print_impl_recur(std::stringstream& out, TName name, TValue value, Args... next) const
    {
        print_impl(out, name, value);
        print_impl_recur(out, next...);
    }

    template <typename... args>
    void print(const std::string& name, std::stringstream& out, args... to_print) const
    {
        out.str("");  // clear contents.

        // Add title.
        out.width(kTotalWidth);
        size_t center = (kTotalWidth - name.size() - 2u) / 2u;  // -2u for ' ' chars
        out << '\n'
            << std::string(center, '*').c_str() << ' ' << name.c_str() << ' '
            << std::string(center - 1, '*').c_str() << '\n';

        // Add columns' headers.
        out.width(kNameWidth);  // Remove hardcoded, need to pre-calc width.
        out.setf(std::ios::left, std::ios::adjustfield);
        out << "Name";
        out.setf(std::ios::right, std::ios::adjustfield);
        out.width(kValueWidth);
        out << "Value\n";

        // Add horizontal separator
        out.width(kTotalWidth);  // Remove hardcoded, need to pre-calc width.
        out << std::setfill('-') << "\n";

        // Reset fill to dots
        out << std::setfill('.');
        print_impl_recur(out, to_print...);

        // Add horizontal separator
        out.width(kTotalWidth);  // Remove hardcoded, need to pre-calc width.
        out << std::setfill('-') << "\n";
    }

    static constexpr size_t kTotalWidth = 61;

    static constexpr size_t kNameWidth = 40;

    static constexpr size_t kValueWidth = 20;
};