def expand_template_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.out,
        substitutions = ctx.attr.substitutions,
    )

_expand_template = rule(
    implementation = expand_template_impl,
    attrs = {
        "template": attr.label(mandatory = True, allow_single_file = True),
        "substitutions": attr.string_dict(mandatory = True),
        "out": attr.output(mandatory = True),
    },
    output_to_genfiles = True,
)

def expand_template(name, template, substitutions, out):
    """Template expansion
    This performs a simple search over the template file for the keys in substitutions,
Vertexwahn marked this conversation as resolved.
    and replaces them with the corresponding values.
    There is no special syntax for the keys.
    To avoid conflicts, you would need to explicitly add delimiters to the key strings, for example "{KEY}" or "@KEY@".
    Args:
      name: The name of the rule.
      template: The template file to expand
      out: The destination of the expanded file
      substitutions: A dictionary mapping strings to their substitutions
    """
    _expand_template(
        name = name,
        template = template,
        substitutions = substitutions,
        out = out,
    )