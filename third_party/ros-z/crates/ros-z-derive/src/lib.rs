//! Derive macros for ros-z traits.
//!
//! Provides:
//! - `MessageTypeInfo` for Rust-native message schema generation
//! - `FromPyMessage` and `IntoPyMessage` for Python bridge conversion

#![allow(clippy::collapsible_if)]

use proc_macro::TokenStream;
use quote::quote;
use syn::{
    Attribute, Data, DeriveInput, Expr, Fields, GenericArgument, GenericParam, Generics, Ident,
    LitStr, PathArguments, Type, parse_macro_input, parse_quote,
};

type TokenStream2 = proc_macro2::TokenStream;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MessageDeriveFlavor {
    Standard,
    Extended,
}

impl MessageDeriveFlavor {
    fn derive_name(self) -> &'static str {
        match self {
            Self::Standard => "MessageTypeInfo",
            Self::Extended => "ExtendedMessageTypeInfo",
        }
    }

    fn type_name_error(self) -> &'static str {
        match self {
            Self::Standard => {
                "MessageTypeInfo derive requires #[ros_msg(type_name = \"my_pkg/msg/MyType\")]"
            }
            Self::Extended => {
                "ExtendedMessageTypeInfo derive requires #[ros_msg(type_name = \"my_pkg/msg/MyType\")]"
            }
        }
    }

    fn tuple_struct_error(self) -> &'static str {
        match self {
            Self::Standard => "MessageTypeInfo derive does not support tuple structs in v1",
            Self::Extended => "ExtendedMessageTypeInfo derive does not support tuple structs in v1",
        }
    }

    fn unit_struct_error(self) -> &'static str {
        match self {
            Self::Standard => "MessageTypeInfo derive does not support unit structs in v1",
            Self::Extended => "ExtendedMessageTypeInfo derive does not support unit structs in v1",
        }
    }

    fn named_struct_only_error(self) -> &'static str {
        match self {
            Self::Standard => "MessageTypeInfo derive only supports named structs in v1",
            Self::Extended => {
                "ExtendedMessageTypeInfo derive only supports named structs and enums in v1"
            }
        }
    }

    fn union_error(self) -> &'static str {
        match self {
            Self::Standard => "MessageTypeInfo derive does not support unions",
            Self::Extended => "ExtendedMessageTypeInfo derive does not support unions",
        }
    }
}

/// Derive macro for implementing ros-z message metadata and dynamic schema generation.
///
/// # Example
/// ```ignore
/// #[derive(MessageTypeInfo)]
/// #[ros_msg(type_name = "custom_msgs/msg/RobotStatus")]
/// pub struct RobotStatus {
///     pub battery_percentage: f64,
///     pub is_moving: bool,
/// }
/// ```
#[proc_macro_derive(MessageTypeInfo, attributes(ros_msg))]
pub fn derive_message_type_info(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match impl_standard_message_type_info(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

/// Derive macro for implementing ros-z extended message schema generation.
#[proc_macro_derive(ExtendedMessageTypeInfo, attributes(ros_msg))]
pub fn derive_extended_message_type_info(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match impl_message_type_info(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

/// Derive macro for extracting Rust messages from Python objects.
///
/// # Example
/// ```ignore
/// #[derive(FromPyMessage)]
/// #[ros_msg(module = "ros_z_msgs_py.types.std_msgs")]
/// pub struct String {
///     pub data: std::string::String,
/// }
/// ```
#[proc_macro_derive(FromPyMessage, attributes(ros_msg))]
pub fn derive_from_py_message(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match impl_from_py_message(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

/// Derive macro for constructing Python objects from Rust messages.
///
/// # Example
/// ```ignore
/// #[derive(IntoPyMessage)]
/// #[ros_msg(module = "ros_z_msgs_py.types.std_msgs")]
/// pub struct String {
///     pub data: std::string::String,
/// }
/// ```
#[proc_macro_derive(IntoPyMessage, attributes(ros_msg))]
pub fn derive_into_py_message(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match impl_into_py_message(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

/// Derive macro for generating config field metadata.
#[proc_macro_derive(ConfigMetadata, attributes(config))]
pub fn derive_config_metadata(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match impl_config_metadata(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

fn impl_message_type_info(input: &DeriveInput) -> syn::Result<TokenStream2> {
    impl_message_type_info_with_flavor(input, MessageDeriveFlavor::Extended)
}

fn impl_standard_message_type_info(input: &DeriveInput) -> syn::Result<TokenStream2> {
    impl_message_type_info_with_flavor(input, MessageDeriveFlavor::Standard)
}

fn impl_message_type_info_with_flavor(
    input: &DeriveInput,
    flavor: MessageDeriveFlavor,
) -> syn::Result<TokenStream2> {
    let name = &input.ident;
    let attrs = parse_ros_msg_args(&input.attrs)?;
    let canonical_type_name = attrs
        .type_name
        .ok_or_else(|| syn::Error::new_spanned(input, flavor.type_name_error()))?;
    let type_name_lit = LitStr::new(&canonical_type_name, proc_macro2::Span::call_site());
    let (package, _kind, message_name) = parse_canonical_type_name(&canonical_type_name)?;
    let package_lit = LitStr::new(&package, proc_macro2::Span::call_site());
    let message_name_lit = LitStr::new(&message_name, proc_macro2::Span::call_site());

    match &input.data {
        Data::Struct(data) => impl_message_type_info_for_struct(
            input,
            data,
            &type_name_lit,
            &package_lit,
            &message_name_lit,
            flavor,
        ),
        Data::Enum(data) => {
            if flavor == MessageDeriveFlavor::Standard {
                return Err(syn::Error::new_spanned(
                    input,
                    flavor.named_struct_only_error(),
                ));
            }

            ensure_non_generic_enum(input, flavor)?;
            impl_message_type_info_for_enum(
                name,
                data,
                &type_name_lit,
                &package_lit,
                &message_name_lit,
            )
        }
        Data::Union(_) => Err(syn::Error::new_spanned(input, flavor.union_error())),
    }
}

fn impl_message_type_info_for_struct(
    input: &DeriveInput,
    data: &syn::DataStruct,
    type_name_lit: &LitStr,
    package_lit: &LitStr,
    message_name_lit: &LitStr,
    flavor: MessageDeriveFlavor,
) -> syn::Result<TokenStream2> {
    ensure_supported_struct_generics(input, flavor)?;
    let name = &input.ident;

    let Fields::Named(fields) = &data.fields else {
        let message = match &data.fields {
            Fields::Unnamed(_) => flavor.tuple_struct_error(),
            Fields::Unit => flavor.unit_struct_error(),
            Fields::Named(_) => unreachable!(),
        };
        return Err(syn::Error::new_spanned(name, message));
    };

    let schema_fields = fields
        .named
        .iter()
        .map(|field| generate_message_field_schema_tokens(field, flavor))
        .collect::<syn::Result<Vec<_>>>()?;

    let bounded_generics = add_field_type_info_bounds(&input.generics);
    let (impl_generics, ty_generics, where_clause) = bounded_generics.split_for_impl();
    let type_params = input
        .generics
        .params
        .iter()
        .filter_map(|param| match param {
            GenericParam::Type(type_param) => Some(&type_param.ident),
            _ => None,
        })
        .collect::<Vec<_>>();
    let generic_arg_names = type_params
        .iter()
        .map(|ident| quote! { <#ident as ::ros_z::FieldTypeInfo>::generic_arg_name() })
        .collect::<Vec<_>>();

    let hash_helper = match flavor {
        MessageDeriveFlavor::Standard => quote! {
            fn __ros_z_type_hash() -> ::ros_z::entity::TypeHash {
                static TYPE_HASH: ::std::sync::OnceLock<
                    ::std::sync::Mutex<
                        ::std::collections::HashMap<::std::any::TypeId, ::ros_z::entity::TypeHash>
                    >
                > =
                    ::std::sync::OnceLock::new();

                let key = ::std::any::TypeId::of::<Self>();
                let cache = TYPE_HASH.get_or_init(|| {
                    ::std::sync::Mutex::new(::std::collections::HashMap::new())
                });
                if let Some(hash) = cache.lock().expect("type hash cache poisoned").get(&key).cloned() {
                    return hash;
                }

                let hash = {
                    use ::ros_z::dynamic::MessageSchemaTypeDescription;

                    Self::__ros_z_schema()
                        .compute_type_hash()
                        .expect("standard-compatible derived message schema must produce a type hash")
                };

                cache.lock().expect("type hash cache poisoned").insert(key, hash.clone());
                hash
            }
        },
        MessageDeriveFlavor::Extended => quote! {
            fn __ros_z_type_hash() -> ::ros_z::entity::TypeHash {
                static TYPE_HASH: ::std::sync::OnceLock<
                    ::std::sync::Mutex<
                        ::std::collections::HashMap<::std::any::TypeId, ::ros_z::entity::TypeHash>
                    >
                > =
                    ::std::sync::OnceLock::new();

                let key = ::std::any::TypeId::of::<Self>();
                let cache = TYPE_HASH.get_or_init(|| {
                    ::std::sync::Mutex::new(::std::collections::HashMap::new())
                });
                if let Some(hash) = cache.lock().expect("type hash cache poisoned").get(&key).cloned() {
                    return hash;
                }

                let hash = {
                    let schema = Self::__ros_z_schema();
                    if schema.uses_extended_types() {
                        ::ros_z::extended_schema::compute_extended_type_hash(&schema)
                            .expect("extended message schema must produce a type hash")
                    } else {
                        use ::ros_z::dynamic::MessageSchemaTypeDescription;

                        schema
                            .compute_type_hash()
                            .expect("standard-compatible extended schema must produce a standard type hash")
                    }
                };

                cache.lock().expect("type hash cache poisoned").insert(key, hash.clone());
                hash
            }
        },
    };

    let extended_trait_impl = if flavor == MessageDeriveFlavor::Extended {
        quote! {
            impl #impl_generics ::ros_z::ExtendedMessageTypeInfo for #name #ty_generics #where_clause {
                fn extended_message_schema() -> ::std::sync::Arc<::ros_z::dynamic::MessageSchema> {
                    Self::__ros_z_schema()
                }
            }
        }
    } else {
        quote! {}
    };

    let field_type_impl = if flavor == MessageDeriveFlavor::Extended {
        quote! {
            fn field_type() -> ::ros_z::dynamic::FieldType {
                ::ros_z::dynamic::FieldType::Message(Self::__ros_z_schema())
            }
        }
    } else {
        quote! {}
    };

    let message_schema_impl = if flavor == MessageDeriveFlavor::Extended {
        quote! {
            fn message_schema() -> Option<::std::sync::Arc<::ros_z::dynamic::MessageSchema>> {
                let schema = Self::__ros_z_schema();
                if schema.uses_extended_types() {
                    None
                } else {
                    Some(schema)
                }
            }

            fn register_type_extensions(node: &::ros_z::node::ZNode) -> ::std::result::Result<(), ::std::string::String> {
                let schema = Self::__ros_z_schema();
                if schema.uses_extended_types() {
                    ::ros_z::extended_schema::register_type::<Self>(node)
                } else {
                    Ok(())
                }
            }
        }
    } else {
        quote! {
            fn message_schema() -> Option<::std::sync::Arc<::ros_z::dynamic::MessageSchema>> {
                let schema = Self::__ros_z_schema();
                if schema.uses_extended_types() {
                    None
                } else {
                    Some(schema)
                }
            }
        }
    };

    Ok(quote! {
        impl #impl_generics #name #ty_generics #where_clause {
            fn __ros_z_type_name() -> &'static str {
                static TYPE_NAME: ::std::sync::OnceLock<
                    ::std::sync::Mutex<
                        ::std::collections::HashMap<::std::any::TypeId, &'static str>
                    >
                > = ::std::sync::OnceLock::new();

                let key = ::std::any::TypeId::of::<Self>();
                let cache = TYPE_NAME.get_or_init(|| {
                    ::std::sync::Mutex::new(::std::collections::HashMap::new())
                });
                if let Some(type_name) = cache.lock().expect("type name cache poisoned").get(&key).copied() {
                    return type_name;
                }

                let generic_arg_names = ::std::vec![#(#generic_arg_names),*];
                let type_name = ::ros_z::format_generic_message_type_name(#type_name_lit, &generic_arg_names);
                let type_name = ::std::boxed::Box::leak(type_name.into_boxed_str());
                cache.lock().expect("type name cache poisoned").insert(key, type_name);
                type_name
            }

            fn __ros_z_schema() -> ::std::sync::Arc<::ros_z::dynamic::MessageSchema> {
                static SCHEMA: ::std::sync::OnceLock<
                    ::std::sync::Mutex<
                        ::std::collections::HashMap<
                            ::std::any::TypeId,
                            ::std::sync::Arc<::ros_z::dynamic::MessageSchema>
                        >
                    >
                > =
                    ::std::sync::OnceLock::new();

                let key = ::std::any::TypeId::of::<Self>();
                let cache = SCHEMA.get_or_init(|| {
                    ::std::sync::Mutex::new(::std::collections::HashMap::new())
                });
                if let Some(schema) = cache.lock().expect("schema cache poisoned").get(&key).cloned() {
                    return schema;
                }

                let type_name = Self::__ros_z_type_name();
                let schema = ::std::sync::Arc::new(::ros_z::dynamic::MessageSchema {
                    type_name: type_name.to_string(),
                    package: #package_lit.to_string(),
                    name: type_name.rsplit('/').next().unwrap_or(#message_name_lit).to_string(),
                    fields: ::std::vec![#(#schema_fields),*],
                    type_hash: None,
                });
                cache.lock().expect("schema cache poisoned").insert(key, schema.clone());
                schema
            }

            #hash_helper
        }

        #extended_trait_impl

        impl #impl_generics ::ros_z::MessageTypeInfo for #name #ty_generics #where_clause {
            fn type_name() -> &'static str {
                Self::__ros_z_type_name()
            }

            fn type_hash() -> ::ros_z::entity::TypeHash {
                Self::__ros_z_type_hash()
            }

            #field_type_impl

            #message_schema_impl
        }

        impl #impl_generics ::ros_z::WithTypeInfo for #name #ty_generics #where_clause {}
    })
}

fn impl_message_type_info_for_enum(
    name: &Ident,
    data: &syn::DataEnum,
    type_name_lit: &LitStr,
    package_lit: &LitStr,
    message_name_lit: &LitStr,
) -> syn::Result<TokenStream2> {
    let message_type_hash_impl = quote! {
        fn type_hash() -> ::ros_z::entity::TypeHash {
            static TYPE_HASH: ::std::sync::OnceLock<::ros_z::entity::TypeHash> =
                ::std::sync::OnceLock::new();

            TYPE_HASH
                .get_or_init(|| {
                    let schema = <Self as ::ros_z::ExtendedMessageTypeInfo>::extended_message_schema();
                    if schema.uses_extended_types() {
                        ::ros_z::extended_schema::compute_extended_type_hash(&schema)
                            .expect("extended message schema must produce a type hash")
                    } else {
                        use ::ros_z::dynamic::MessageSchemaTypeDescription;

                        schema
                            .compute_type_hash()
                            .expect("standard-compatible extended schema must produce a standard type hash")
                    }
                })
                .clone()
        }
    };

    if data.variants.is_empty() {
        return Err(syn::Error::new_spanned(
            name,
            "ExtendedMessageTypeInfo derive requires enums to have at least one variant",
        ));
    }

    let variant_tokens = data
        .variants
        .iter()
        .map(generate_enum_variant_schema_tokens)
        .collect::<syn::Result<Vec<_>>>()?;

    Ok(quote! {
        impl #name {
            fn __ros_z_enum_schema() -> ::std::sync::Arc<::ros_z::dynamic::EnumSchema> {
                static ENUM_SCHEMA: ::std::sync::OnceLock<::std::sync::Arc<::ros_z::dynamic::EnumSchema>> =
                    ::std::sync::OnceLock::new();

                ENUM_SCHEMA
                    .get_or_init(|| {
                        ::std::sync::Arc::new(::ros_z::dynamic::EnumSchema {
                            type_name: #type_name_lit.to_string(),
                            variants: ::std::vec![#(#variant_tokens),*],
                        })
                    })
                    .clone()
            }
        }

        impl ::ros_z::ExtendedMessageTypeInfo for #name {
            fn extended_message_schema() -> ::std::sync::Arc<::ros_z::dynamic::MessageSchema> {
                static SCHEMA: ::std::sync::OnceLock<::std::sync::Arc<::ros_z::dynamic::MessageSchema>> =
                    ::std::sync::OnceLock::new();

                SCHEMA
                    .get_or_init(|| {
                        ::std::sync::Arc::new(::ros_z::dynamic::MessageSchema {
                            type_name: #type_name_lit.to_string(),
                            package: #package_lit.to_string(),
                            name: #message_name_lit.to_string(),
                            fields: ::std::vec![
                                ::ros_z::dynamic::FieldSchema::new(
                                    "value",
                                    ::ros_z::dynamic::FieldType::Enum(Self::__ros_z_enum_schema()),
                                )
                            ],
                            type_hash: None,
                        })
                    })
                    .clone()
            }

            fn extended_field_type() -> ::ros_z::dynamic::FieldType {
                ::ros_z::dynamic::FieldType::Enum(Self::__ros_z_enum_schema())
            }
        }

        impl ::ros_z::MessageTypeInfo for #name {
            fn type_name() -> &'static str {
                #type_name_lit
            }

            #message_type_hash_impl

            fn field_type() -> ::ros_z::dynamic::FieldType {
                ::ros_z::dynamic::FieldType::Enum(Self::__ros_z_enum_schema())
            }

            fn message_schema() -> Option<::std::sync::Arc<::ros_z::dynamic::MessageSchema>> {
                None
            }

            fn register_type_extensions(node: &::ros_z::node::ZNode) -> ::std::result::Result<(), ::std::string::String> {
                ::ros_z::extended_schema::register_type::<Self>(node)
            }
        }

        impl ::ros_z::WithTypeInfo for #name {}
    })
}

fn ensure_supported_struct_generics(
    input: &DeriveInput,
    flavor: MessageDeriveFlavor,
) -> syn::Result<()> {
    for param in &input.generics.params {
        match param {
            GenericParam::Type(_) => {}
            GenericParam::Lifetime(lifetime) => {
                return Err(syn::Error::new_spanned(
                    lifetime,
                    format!(
                        "{} derive does not support lifetime parameters in v1",
                        flavor.derive_name()
                    ),
                ));
            }
            GenericParam::Const(const_param) => {
                return Err(syn::Error::new_spanned(
                    const_param,
                    format!(
                        "{} derive does not support const generics in v1",
                        flavor.derive_name()
                    ),
                ));
            }
        }
    }

    Ok(())
}

fn ensure_non_generic_enum(input: &DeriveInput, flavor: MessageDeriveFlavor) -> syn::Result<()> {
    if input.generics.params.is_empty() {
        return Ok(());
    }

    Err(syn::Error::new_spanned(
        &input.generics,
        format!(
            "{} derive does not support generic enums in v1",
            flavor.derive_name()
        ),
    ))
}

fn add_field_type_info_bounds(generics: &Generics) -> Generics {
    let mut bounded = generics.clone();
    for param in &mut bounded.params {
        if let GenericParam::Type(type_param) = param {
            type_param.bounds.push(parse_quote!(::ros_z::FieldTypeInfo));
            type_param.bounds.push(parse_quote!('static));
        }
    }
    bounded
}

fn impl_from_py_message(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let name = &input.ident;

    let Data::Struct(ref data) = input.data else {
        return Err(syn::Error::new_spanned(
            input,
            "FromPyMessage only supports structs",
        ));
    };

    let Fields::Named(ref fields) = data.fields else {
        return Err(syn::Error::new_spanned(
            input,
            "FromPyMessage requires named fields",
        ));
    };

    let field_extractions: Vec<TokenStream2> = fields
        .named
        .iter()
        .map(|f| {
            let field_name = f.ident.as_ref().unwrap();
            let field_name_str = field_ident_to_config_path(field_name);
            let field_type = &f.ty;
            let use_zbuf = parse_ros_msg_args(&f.attrs)?.zbuf;
            generate_field_extraction(field_name, &field_name_str, field_type, use_zbuf)
        })
        .collect::<syn::Result<Vec<_>>>()?;

    Ok(quote! {
        impl ::ros_z::python_bridge::FromPyMessage for #name {
            fn from_py(obj: &::pyo3::Bound<'_, ::pyo3::PyAny>) -> ::pyo3::PyResult<Self> {
                use ::pyo3::types::PyAnyMethods;
                Ok(Self {
                    #(#field_extractions),*
                })
            }
        }
    })
}

fn impl_into_py_message(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let name = &input.ident;

    let Data::Struct(ref data) = input.data else {
        return Err(syn::Error::new_spanned(
            input,
            "IntoPyMessage only supports structs",
        ));
    };

    let Fields::Named(ref fields) = data.fields else {
        return Err(syn::Error::new_spanned(
            input,
            "IntoPyMessage requires named fields",
        ));
    };

    let module_path = extract_module_path(&input.attrs)?;

    let field_constructions: Vec<TokenStream2> = fields
        .named
        .iter()
        .map(|f| {
            let field_name = f.ident.as_ref().unwrap();
            let field_name_str = field_ident_to_config_path(field_name);
            let field_type = &f.ty;
            let use_zbuf = parse_ros_msg_args(&f.attrs)?.zbuf;
            generate_field_construction(field_name, &field_name_str, field_type, use_zbuf)
        })
        .collect::<syn::Result<Vec<_>>>()?;

    let name_str = name.to_string();

    Ok(quote! {
        impl ::ros_z::python_bridge::IntoPyMessage for #name {
            fn into_py_message(&self, py: ::pyo3::Python) -> ::pyo3::PyResult<::pyo3::PyObject> {
                use ::pyo3::types::{PyAnyMethods, PyDictMethods, PyModuleMethods};
                let module = ::pyo3::types::PyModule::import_bound(py, #module_path)?;
                let class = module.getattr(#name_str)?;

                let kwargs = ::pyo3::types::PyDict::new_bound(py);
                #(#field_constructions)*

                class.call((), Some(&kwargs)).map(|obj| obj.into())
            }
        }
    })
}

fn generate_message_field_schema_tokens(
    field: &syn::Field,
    flavor: MessageDeriveFlavor,
) -> syn::Result<TokenStream2> {
    let field_name = field
        .ident
        .as_ref()
        .ok_or_else(|| syn::Error::new_spanned(field, "named fields are required"))?;
    let field_name_str = field_ident_to_config_path(field_name);
    let field_type = generate_message_field_type_tokens(&field.ty, flavor)?;

    Ok(quote! {
        ::ros_z::dynamic::FieldSchema::new(#field_name_str, #field_type)
    })
}

fn generate_message_field_type_tokens(
    ty: &Type,
    flavor: MessageDeriveFlavor,
) -> syn::Result<TokenStream2> {
    match ty {
        Type::Path(type_path) => {
            if type_path.qself.is_some() {
                return unsupported_message_type(
                    ty,
                    match flavor {
                        MessageDeriveFlavor::Standard => {
                            "qualified self types are not supported in v1"
                        }
                        MessageDeriveFlavor::Extended => {
                            "qualified self types are not supported by ExtendedMessageTypeInfo derive in v1"
                        }
                    },
                );
            }

            let last_segment = type_path.path.segments.last().ok_or_else(|| {
                syn::Error::new_spanned(
                    ty,
                    match flavor {
                        MessageDeriveFlavor::Standard => {
                            "unsupported field type for MessageTypeInfo derive"
                        }
                        MessageDeriveFlavor::Extended => {
                            "unsupported field type for ExtendedMessageTypeInfo derive"
                        }
                    },
                )
            })?;
            let ident_str = last_segment.ident.to_string();

            match ident_str.as_str() {
                "bool" => Ok(quote! { ::ros_z::dynamic::FieldType::Bool }),
                "i8" => Ok(quote! { ::ros_z::dynamic::FieldType::Int8 }),
                "u8" => Ok(quote! { ::ros_z::dynamic::FieldType::Uint8 }),
                "i16" => Ok(quote! { ::ros_z::dynamic::FieldType::Int16 }),
                "u16" => Ok(quote! { ::ros_z::dynamic::FieldType::Uint16 }),
                "i32" => Ok(quote! { ::ros_z::dynamic::FieldType::Int32 }),
                "u32" => Ok(quote! { ::ros_z::dynamic::FieldType::Uint32 }),
                "i64" => Ok(quote! { ::ros_z::dynamic::FieldType::Int64 }),
                "u64" => Ok(quote! { ::ros_z::dynamic::FieldType::Uint64 }),
                "f32" => Ok(quote! { ::ros_z::dynamic::FieldType::Float32 }),
                "f64" => Ok(quote! { ::ros_z::dynamic::FieldType::Float64 }),
                "String" => Ok(quote! { ::ros_z::dynamic::FieldType::String }),
                "usize" | "isize" => unsupported_message_type(
                    ty,
                    match flavor {
                        MessageDeriveFlavor::Standard => {
                            "usize and isize are not supported by MessageTypeInfo derive in v1"
                        }
                        MessageDeriveFlavor::Extended => {
                            "usize and isize are not supported by ExtendedMessageTypeInfo derive in v1"
                        }
                    },
                ),
                "HashMap" | "BTreeMap" => unsupported_message_type(
                    ty,
                    match flavor {
                        MessageDeriveFlavor::Standard => {
                            "map fields are not supported by MessageTypeInfo derive in v1"
                        }
                        MessageDeriveFlavor::Extended => {
                            "map fields are not supported by ExtendedMessageTypeInfo derive in v1"
                        }
                    },
                ),
                "Option" if flavor == MessageDeriveFlavor::Standard => unsupported_message_type(
                    ty,
                    "Option fields are not supported by MessageTypeInfo derive in v1",
                ),
                "Option" => {
                    let PathArguments::AngleBracketed(args) = &last_segment.arguments else {
                        return unsupported_message_type(
                            ty,
                            "Option fields must specify an inner type",
                        );
                    };
                    let Some(GenericArgument::Type(inner)) = args.args.first() else {
                        return unsupported_message_type(
                            ty,
                            "Option fields must specify an inner type",
                        );
                    };
                    let inner_tokens = generate_message_field_type_tokens(inner, flavor)?;
                    Ok(quote! {
                        ::ros_z::dynamic::FieldType::Optional(::std::boxed::Box::new(#inner_tokens))
                    })
                }
                "Vec" => {
                    let PathArguments::AngleBracketed(args) = &last_segment.arguments else {
                        return unsupported_message_type(
                            ty,
                            "Vec fields must specify an element type",
                        );
                    };
                    let Some(GenericArgument::Type(inner)) = args.args.first() else {
                        return unsupported_message_type(
                            ty,
                            "Vec fields must specify an element type",
                        );
                    };
                    let inner_tokens = generate_message_field_type_tokens(inner, flavor)?;
                    Ok(quote! {
                        ::ros_z::dynamic::FieldType::Sequence(::std::boxed::Box::new(#inner_tokens))
                    })
                }
                _ => Ok(quote! {
                    <#ty as ::ros_z::FieldTypeInfo>::field_type()
                }),
            }
        }
        Type::Array(array) => {
            let len = match &array.len {
                Expr::Lit(expr_lit) => match &expr_lit.lit {
                    syn::Lit::Int(value) => value.base10_parse::<usize>()?,
                    _ => {
                        return unsupported_message_type(
                            ty,
                            "array lengths must be integer literals for MessageTypeInfo derive",
                        );
                    }
                },
                _ => {
                    return unsupported_message_type(
                        ty,
                        "array lengths must be integer literals for MessageTypeInfo derive",
                    );
                }
            };

            let inner_tokens = generate_message_field_type_tokens(&array.elem, flavor)?;
            Ok(quote! {
                ::ros_z::dynamic::FieldType::Array(::std::boxed::Box::new(#inner_tokens), #len)
            })
        }
        Type::Tuple(_) => unsupported_message_type(
            ty,
            match flavor {
                MessageDeriveFlavor::Standard => {
                    "tuple fields are not supported by MessageTypeInfo derive in v1"
                }
                MessageDeriveFlavor::Extended => {
                    "tuple fields are not supported by ExtendedMessageTypeInfo derive in v1"
                }
            },
        ),
        _ => unsupported_message_type(
            ty,
            match flavor {
                MessageDeriveFlavor::Standard => {
                    "unsupported field type for MessageTypeInfo derive in v1"
                }
                MessageDeriveFlavor::Extended => {
                    "unsupported field type for ExtendedMessageTypeInfo derive in v1"
                }
            },
        ),
    }
}

fn generate_enum_variant_schema_tokens(variant: &syn::Variant) -> syn::Result<TokenStream2> {
    let variant_name = variant.ident.to_string();
    let payload = match &variant.fields {
        Fields::Unit => quote! { ::ros_z::dynamic::EnumPayloadSchema::Unit },
        Fields::Unnamed(fields) if fields.unnamed.len() == 1 => {
            let field_type = generate_message_field_type_tokens(
                &fields.unnamed[0].ty,
                MessageDeriveFlavor::Extended,
            )?;
            quote! {
                ::ros_z::dynamic::EnumPayloadSchema::Newtype(::std::boxed::Box::new(#field_type))
            }
        }
        Fields::Unnamed(fields) => {
            let field_types = fields
                .unnamed
                .iter()
                .map(|field| {
                    generate_message_field_type_tokens(&field.ty, MessageDeriveFlavor::Extended)
                })
                .collect::<syn::Result<Vec<_>>>()?;
            quote! {
                ::ros_z::dynamic::EnumPayloadSchema::Tuple(::std::vec![#(#field_types),*])
            }
        }
        Fields::Named(fields) => {
            let field_schemas = fields
                .named
                .iter()
                .map(|field| {
                    generate_message_field_schema_tokens(field, MessageDeriveFlavor::Extended)
                })
                .collect::<syn::Result<Vec<_>>>()?;
            quote! {
                ::ros_z::dynamic::EnumPayloadSchema::Struct(::std::vec![#(#field_schemas),*])
            }
        }
    };

    Ok(quote! {
        ::ros_z::dynamic::EnumVariantSchema::new(#variant_name, #payload)
    })
}

fn unsupported_message_type<T>(node: &T, message: &str) -> syn::Result<TokenStream2>
where
    T: quote::ToTokens,
{
    Err(syn::Error::new_spanned(node, message))
}

fn parse_canonical_type_name(type_name: &str) -> syn::Result<(String, String, String)> {
    let parts: Vec<_> = type_name.split('/').collect();
    if parts.len() != 3 {
        return Err(syn::Error::new(
            proc_macro2::Span::call_site(),
            "ros_msg type_name must look like \"my_pkg/msg/MyType\"",
        ));
    }

    match parts[1] {
        "msg" | "srv" | "action" => Ok((
            parts[0].to_string(),
            parts[1].to_string(),
            parts[2].to_string(),
        )),
        _ => Err(syn::Error::new(
            proc_macro2::Span::call_site(),
            "ros_msg type_name kind must be one of: msg, srv, action",
        )),
    }
}

/// Generate extraction code for a single field.
fn generate_field_extraction(
    field_name: &Ident,
    field_name_str: &str,
    field_type: &Type,
    use_zbuf: bool,
) -> syn::Result<TokenStream2> {
    if use_zbuf {
        return Ok(quote! {
            #field_name: {
                use ::pyo3::types::{PyByteArrayMethods, PyBytesMethods};
                let py_attr = obj.getattr(#field_name_str)?;
                if let Ok(view) = py_attr.downcast::<::ros_z::zbuf_view::ZBufView>() {
                    view.borrow().zbuf().clone()
                } else if let Ok(bytes) = py_attr.downcast::<::pyo3::types::PyBytes>() {
                    ::ros_z::ZBuf::from(bytes.as_bytes().to_vec())
                } else if let Ok(bytearray) = py_attr.downcast::<::pyo3::types::PyByteArray>() {
                    ::ros_z::ZBuf::from(unsafe { bytearray.as_bytes() }.to_vec())
                } else {
                    let bytes: Vec<u8> = py_attr.extract()?;
                    ::ros_z::ZBuf::from(bytes)
                }
            }
        });
    }

    match classify_type(field_type) {
        TypeClass::Primitive | TypeClass::String => Ok(quote! {
            #field_name: obj.getattr(#field_name_str)?.extract()?
        }),
        TypeClass::Vec(inner) => {
            let inner_class = classify_type(&inner);
            match inner_class {
                TypeClass::Primitive if is_u8_type(&inner) => Ok(quote! {
                    #field_name: {
                        use ::pyo3::types::{PyByteArrayMethods, PyBytesMethods};
                        let py_attr = obj.getattr(#field_name_str)?;
                        if let Ok(bytes) = py_attr.downcast::<::pyo3::types::PyBytes>() {
                            bytes.as_bytes().to_vec()
                        } else if let Ok(bytearray) = py_attr.downcast::<::pyo3::types::PyByteArray>() {
                            unsafe { bytearray.as_bytes() }.to_vec()
                        } else {
                            py_attr.extract()?
                        }
                    }
                }),
                TypeClass::Primitive | TypeClass::String => Ok(quote! {
                    #field_name: obj.getattr(#field_name_str)?.extract()?
                }),
                _ => Ok(quote! {
                    #field_name: {
                        use ::pyo3::types::PyListMethods;
                        let py_list = obj.getattr(#field_name_str)?;
                        let mut vec = Vec::new();
                        for item in py_list.iter()? {
                            vec.push(<#inner as ::ros_z::python_bridge::FromPyMessage>::from_py(&item?)?);
                        }
                        vec
                    }
                }),
            }
        }
        TypeClass::Array(inner, size) => {
            let inner_class = classify_type(&inner);
            match inner_class {
                TypeClass::Primitive | TypeClass::String => Ok(quote! {
                    #field_name: {
                        let v: Vec<_> = obj.getattr(#field_name_str)?.extract()?;
                        let mut arr: #field_type = unsafe { ::std::mem::zeroed() };
                        let len = ::std::cmp::min(v.len(), #size);
                        arr[..len].copy_from_slice(&v[..len]);
                        arr
                    }
                }),
                _ => Ok(quote! {
                    #field_name: {
                        use ::pyo3::types::PyListMethods;
                        let py_list = obj.getattr(#field_name_str)?;
                        let mut arr: #field_type = ::std::array::from_fn(|_| Default::default());
                        for (i, item) in py_list.iter()?.enumerate().take(#size) {
                            arr[i] = <#inner as ::ros_z::python_bridge::FromPyMessage>::from_py(&item?)?;
                        }
                        arr
                    }
                }),
            }
        }
        TypeClass::Nested => Ok(quote! {
            #field_name: {
                let py_attr = obj.getattr(#field_name_str)?;
                if py_attr.is_none() {
                    Default::default()
                } else {
                    <#field_type as ::ros_z::python_bridge::FromPyMessage>::from_py(&py_attr)?
                }
            }
        }),
        TypeClass::ZBuf => Ok(quote! {
            #field_name: {
                use ::pyo3::types::{PyByteArrayMethods, PyBytesMethods};
                let py_attr = obj.getattr(#field_name_str)?;
                let bytes: Vec<u8> = if let Ok(bytes) = py_attr.downcast::<::pyo3::types::PyBytes>() {
                    bytes.as_bytes().to_vec()
                } else if let Ok(bytearray) = py_attr.downcast::<::pyo3::types::PyByteArray>() {
                    unsafe { bytearray.as_bytes() }.to_vec()
                } else {
                    py_attr.extract()?
                };
                ::ros_z::ZBuf::from(bytes)
            }
        }),
    }
}

/// Generate construction code for a single field (Rust -> Python).
fn generate_field_construction(
    field_name: &Ident,
    field_name_str: &str,
    field_type: &Type,
    use_zbuf: bool,
) -> syn::Result<TokenStream2> {
    if use_zbuf {
        return Ok(quote! {
            {
                let zbuf_view = ::ros_z::zbuf_view::ZBufView::new(self.#field_name.clone());
                let py_view = ::pyo3::Py::new(py, zbuf_view)?;
                kwargs.set_item(#field_name_str, py_view)?;
            }
        });
    }

    match classify_type(field_type) {
        TypeClass::Primitive => Ok(quote! {
            kwargs.set_item(#field_name_str, self.#field_name)?;
        }),
        TypeClass::String => Ok(quote! {
            kwargs.set_item(#field_name_str, &self.#field_name)?;
        }),
        TypeClass::Vec(inner) => {
            let inner_class = classify_type(&inner);
            match inner_class {
                TypeClass::Primitive if is_u8_type(&inner) => Ok(quote! {
                    {
                        let py_bytes = ::pyo3::types::PyBytes::new_bound(py, &self.#field_name);
                        kwargs.set_item(#field_name_str, py_bytes)?;
                    }
                }),
                TypeClass::Primitive | TypeClass::String => Ok(quote! {
                    kwargs.set_item(#field_name_str, &self.#field_name)?;
                }),
                _ => Ok(quote! {
                    {
                        use ::pyo3::types::PyListMethods;
                        let py_list = ::pyo3::types::PyList::empty_bound(py);
                        for item in &self.#field_name {
                            py_list.append(
                                <#inner as ::ros_z::python_bridge::IntoPyMessage>::into_py_message(item, py)?
                            )?;
                        }
                        kwargs.set_item(#field_name_str, py_list)?;
                    }
                }),
            }
        }
        TypeClass::Array(inner, _) => {
            let inner_class = classify_type(&inner);
            match inner_class {
                TypeClass::Primitive | TypeClass::String => Ok(quote! {
                    kwargs.set_item(#field_name_str, self.#field_name.to_vec())?;
                }),
                _ => Ok(quote! {
                    {
                        use ::pyo3::types::PyListMethods;
                        let py_list = ::pyo3::types::PyList::empty_bound(py);
                        for item in &self.#field_name {
                            py_list.append(
                                <#inner as ::ros_z::python_bridge::IntoPyMessage>::into_py_message(item, py)?
                            )?;
                        }
                        kwargs.set_item(#field_name_str, py_list)?;
                    }
                }),
            }
        }
        TypeClass::Nested => Ok(quote! {
            kwargs.set_item(
                #field_name_str,
                <#field_type as ::ros_z::python_bridge::IntoPyMessage>::into_py_message(&self.#field_name, py)?
            )?;
        }),
        TypeClass::ZBuf => Ok(quote! {
            {
                use ::zenoh_buffers::buffer::SplitBuffer;
                let bytes = self.#field_name.contiguous();
                let py_bytes = ::pyo3::types::PyBytes::new_bound(py, bytes.as_ref());
                kwargs.set_item(#field_name_str, py_bytes)?;
            }
        }),
    }
}

/// Type classification for Python conversion code generation.
#[derive(Debug)]
enum TypeClass {
    Primitive,
    String,
    Vec(Box<Type>),
    Array(Box<Type>, usize),
    Nested,
    ZBuf,
}

/// Classify a type for Python conversion code generation purposes.
fn classify_type(ty: &Type) -> TypeClass {
    if let Type::Path(type_path) = ty {
        let segments = &type_path.path.segments;
        if let Some(last_segment) = segments.last() {
            let ident_str = last_segment.ident.to_string();

            if matches!(
                ident_str.as_str(),
                "bool"
                    | "i8"
                    | "u8"
                    | "i16"
                    | "u16"
                    | "i32"
                    | "u32"
                    | "i64"
                    | "u64"
                    | "f32"
                    | "f64"
            ) {
                return TypeClass::Primitive;
            }

            if ident_str == "String" {
                return TypeClass::String;
            }

            if ident_str == "ZBuf" {
                return TypeClass::ZBuf;
            }

            if ident_str == "Vec" {
                if let PathArguments::AngleBracketed(args) = &last_segment.arguments {
                    if let Some(GenericArgument::Type(inner)) = args.args.first() {
                        return TypeClass::Vec(Box::new(inner.clone()));
                    }
                }
            }
        }
    }

    if let Type::Array(arr) = ty {
        if let Expr::Lit(lit) = &arr.len {
            if let syn::Lit::Int(int_lit) = &lit.lit {
                if let Ok(size) = int_lit.base10_parse::<usize>() {
                    return TypeClass::Array(Box::new((*arr.elem).clone()), size);
                }
            }
        }
    }

    TypeClass::Nested
}

#[derive(Default)]
struct RosMsgArgs {
    module: Option<String>,
    type_name: Option<String>,
    zbuf: bool,
}

fn parse_ros_msg_args(attrs: &[Attribute]) -> syn::Result<RosMsgArgs> {
    let mut parsed = RosMsgArgs::default();

    for attr in attrs {
        if !attr.path().is_ident("ros_msg") {
            continue;
        }

        attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("module") {
                let value = meta.value()?.parse::<LitStr>()?;
                parsed.module = Some(value.value());
                return Ok(());
            }

            if meta.path.is_ident("type_name") {
                let value = meta.value()?.parse::<LitStr>()?;
                parsed.type_name = Some(value.value());
                return Ok(());
            }

            if meta.path.is_ident("zbuf") {
                parsed.zbuf = true;
                return Ok(());
            }

            Err(meta
                .error("unsupported ros_msg attribute, expected one of: module, type_name, zbuf"))
        })?;
    }

    Ok(parsed)
}

fn extract_module_path(attrs: &[Attribute]) -> syn::Result<String> {
    Ok(parse_ros_msg_args(attrs)?
        .module
        .unwrap_or_else(|| "ros_z_msgs_py.types".to_string()))
}

fn is_u8_type(ty: &Type) -> bool {
    if let Type::Path(type_path) = ty {
        if let Some(last_segment) = type_path.path.segments.last() {
            return last_segment.ident == "u8";
        }
    }
    false
}

fn field_ident_to_config_path(ident: &Ident) -> String {
    let name = ident.to_string();
    if let Some(stripped) = name.strip_prefix("r#") {
        stripped.to_string()
    } else {
        name
    }
}

fn impl_config_metadata(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let name = &input.ident;

    let Data::Struct(data) = &input.data else {
        return Err(syn::Error::new_spanned(
            input,
            "ConfigMetadata derive only supports structs",
        ));
    };

    let Fields::Named(fields) = &data.fields else {
        return Err(syn::Error::new_spanned(
            input,
            "ConfigMetadata derive only supports named structs",
        ));
    };

    let field_entries = fields
        .named
        .iter()
        .map(generate_config_metadata_field_tokens)
        .collect::<syn::Result<Vec<_>>>()?;

    Ok(quote! {
        impl ::ros_z_config::ConfigMetadata for #name {
            fn config_metadata() -> ::std::vec::Vec<::ros_z_config::ConfigFieldMetadata> {
                let mut fields = ::std::vec::Vec::new();
                #(#field_entries)*
                fields
            }
        }
    })
}

fn generate_config_metadata_field_tokens(field: &syn::Field) -> syn::Result<TokenStream2> {
    let field_ident = field
        .ident
        .as_ref()
        .ok_or_else(|| syn::Error::new_spanned(field, "expected named field"))?;
    let field_name = field_ident_to_config_path(field_ident);
    let field_name_lit = LitStr::new(&field_name, field_ident.span());
    let field_ty = &field.ty;

    let attrs = parse_config_attrs(&field.attrs)?;
    let description = attrs
        .doc
        .unwrap_or_else(|| extract_doc_comment(&field.attrs));
    let description_lit = LitStr::new(&description, field_ident.span());
    let writable = attrs.writable;
    let min_tokens = option_f64_tokens(attrs.min);
    let max_tokens = option_f64_tokens(attrs.max);

    if is_leaf_config_type(field_ty) {
        Ok(quote! {
            fields.push(::ros_z_config::ConfigFieldMetadata {
                path: #field_name_lit.to_string(),
                type_name: ::std::string::String::from(::std::any::type_name::<#field_ty>()),
                description: #description_lit.to_string(),
                writable: #writable,
                min: #min_tokens,
                max: #max_tokens,
            });
        })
    } else {
        Ok(quote! {
            fields.extend(<#field_ty as ::ros_z_config::ConfigMetadata>::config_metadata_prefixed(#field_name_lit));
        })
    }
}

#[derive(Default)]
struct ConfigAttrs {
    doc: Option<String>,
    writable: bool,
    min: Option<f64>,
    max: Option<f64>,
}

fn parse_config_attrs(attrs: &[Attribute]) -> syn::Result<ConfigAttrs> {
    let mut parsed = ConfigAttrs {
        writable: true,
        ..ConfigAttrs::default()
    };

    for attr in attrs {
        if !attr.path().is_ident("config") {
            continue;
        }

        attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("doc") {
                let value = meta.value()?.parse::<LitStr>()?;
                parsed.doc = Some(value.value());
                return Ok(());
            }

            if meta.path.is_ident("writable") {
                let value = meta.value()?.parse::<Expr>()?;
                parsed.writable = parse_bool_expr(&value)?;
                return Ok(());
            }

            if meta.path.is_ident("min") {
                let value = meta.value()?.parse::<Expr>()?;
                parsed.min = Some(parse_f64_expr(&value)?);
                return Ok(());
            }

            if meta.path.is_ident("max") {
                let value = meta.value()?.parse::<Expr>()?;
                parsed.max = Some(parse_f64_expr(&value)?);
                return Ok(());
            }

            Err(meta
                .error("unsupported config attribute, expected one of: doc, writable, min, max"))
        })?;
    }

    Ok(parsed)
}

fn extract_doc_comment(attrs: &[Attribute]) -> String {
    attrs
        .iter()
        .filter(|attr| attr.path().is_ident("doc"))
        .filter_map(|attr| match &attr.meta {
            syn::Meta::NameValue(meta) => match &meta.value {
                Expr::Lit(expr) => match &expr.lit {
                    syn::Lit::Str(value) => Some(value.value().trim().to_string()),
                    _ => None,
                },
                _ => None,
            },
            _ => None,
        })
        .filter(|value| !value.is_empty())
        .collect::<Vec<_>>()
        .join(" ")
}

fn parse_bool_expr(expr: &Expr) -> syn::Result<bool> {
    match expr {
        Expr::Lit(expr_lit) => match &expr_lit.lit {
            syn::Lit::Bool(value) => Ok(value.value),
            _ => Err(syn::Error::new_spanned(expr, "expected boolean literal")),
        },
        _ => Err(syn::Error::new_spanned(expr, "expected boolean literal")),
    }
}

fn parse_f64_expr(expr: &Expr) -> syn::Result<f64> {
    match expr {
        Expr::Lit(expr_lit) => match &expr_lit.lit {
            syn::Lit::Float(value) => value.base10_parse::<f64>(),
            syn::Lit::Int(value) => value.base10_parse::<f64>(),
            _ => Err(syn::Error::new_spanned(expr, "expected numeric literal")),
        },
        Expr::Unary(unary) if matches!(unary.op, syn::UnOp::Neg(_)) => {
            Ok(-parse_f64_expr(&unary.expr)?)
        }
        _ => Err(syn::Error::new_spanned(expr, "expected numeric literal")),
    }
}

fn option_f64_tokens(value: Option<f64>) -> TokenStream2 {
    match value {
        Some(value) => quote!(::std::option::Option::Some(#value)),
        None => quote!(::std::option::Option::None),
    }
}

fn is_leaf_config_type(ty: &Type) -> bool {
    match ty {
        Type::Path(type_path) => {
            let Some(last) = type_path.path.segments.last() else {
                return true;
            };
            matches!(
                last.ident.to_string().as_str(),
                "bool"
                    | "String"
                    | "str"
                    | "u8"
                    | "u16"
                    | "u32"
                    | "u64"
                    | "usize"
                    | "i8"
                    | "i16"
                    | "i32"
                    | "i64"
                    | "isize"
                    | "f32"
                    | "f64"
                    | "Option"
                    | "Vec"
                    | "HashMap"
                    | "BTreeMap"
                    | "PathBuf"
            )
        }
        Type::Array(_) => true,
        _ => true,
    }
}
